#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as ROS_Point
import message_filters

import math
import tf2_ros
import geometry_msgs.msg

import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree

import os

import cv2

DEPTH_SCALE = 1000.0
BASE_FRAME = "freicar_4/base_link"

def order_points(points, ind, max_dist=0.2):
    points_new = [ points.pop(ind) ]  # initialize a new list of points with the known first point
    pcurr      = points_new[-1]       # initialize the current point (as the known point)
    while len(points)>0:
        d      = np.linalg.norm(np.array(points) - np.array(pcurr), axis=1)  # distances between pcurr and all other remaining points
        ind    = d.argmin()                   # index of the closest point
        #if d[ind] < max_dist:
        points_new.append( points.pop(ind) )  # append the closest point to points_new
        #else:
        #    print("WARNING: discarding last {} points".format(len(points)))
        #    break
        pcurr  = points_new[-1]               # update the current point
    return points_new

def color_components(labels):
    # Map component labels to hue val
    label_hue = np.uint8(179*labels/np.max(labels))
    blank_ch = 255*np.ones_like(label_hue)
    labeled_img = cv2.merge([label_hue, blank_ch, blank_ch])

    # cvt to BGR for display
    labeled_img = cv2.cvtColor(labeled_img, cv2.COLOR_HSV2BGR)

    # set bg label to black
    labeled_img[label_hue==0] = 0

    return labeled_img

def subsample_depth(depth, scale_x=2, scale_y=2):
    M, N = depth.shape
    K = scale_y
    L = scale_x

    MK = M // K
    NL = N // L
    return depth[:MK*K, :NL*L].reshape(MK, K, NL, L).min(axis=(1, 3)) # TODO: should handle zeros (invalid depth) differently.
    #like this, output pixel will be zero (invalid) if any measure in kernel is invalid.
    #this should only be the case, if **all** pixel in kernel are invalid.
    #we should choose min of all non-zero pixels

def preprocess(image):
    data = image[:, :, ::-1].astype(np.float32) / float(255.0) # normalize to [0...1]
    #print(data.dtype)
    # Switch from HWC to to CHW order
    return np.moveaxis(data, 2, 0)

def postprocess(image, da_pred, _ll_pred):
    img_rs=image.copy()
    ll_pred = _ll_pred.copy()

    img_rs[da_pred>100]=[255,0,0]
    img_rs[ll_pred>100]=[0,255,0]

    ll_pred[:int(ll_pred.shape[0]*0.6),:] = 0 # use only bottom part of the lines
    ll_pred_thresh = cv2.threshold(ll_pred, 100, 255, cv2.THRESH_BINARY)[1]  # ensure binary
    num_labels, labels_im = cv2.connectedComponents(ll_pred_thresh)
    print("extracted {} connected components from line mask".format(num_labels))
    cc_im = color_components(labels_im)
    #print("labels_im with shape: {}".format(labels_im.shape))

    labels_im_bottom_right = labels_im[(labels_im.shape[0]*2)//3:, (labels_im.shape[1]*2)//3:] # use bottom right area as seed for right limit line
    #print("bottom-right labels_im with shape: {}".format(labels_im_bottom_right.shape))

    label_counts = np.bincount(labels_im_bottom_right.flatten()) # get the largest component in the bottom right area (with a min size, and that starts to the bottom of the image)
    print(label_counts)

    MIN_NumPx_RightLane = 500
    lane_bottom_tol = 0.8
    right_lane_id = None
    id_usable = False
    if len(label_counts) > 1:
        #right_lane_id = np.argmax(label_counts[1:])+1 # exclude zero label (bg)
        sort_idx = np.argsort(label_counts[1:]) + 1 # ascending order

        for i in range(len(sort_idx)-1, -1, -1):
            right_lane_id = sort_idx[i]
            if label_counts[right_lane_id] < MIN_NumPx_RightLane:
                break

            lane_idxs = np.where(labels_im_bottom_right == right_lane_id)

            max_v = lane_idxs[0].max()
            labels_im_br_height = labels_im_bottom_right.shape[0]

            print('current seed: count: {}, max_v: {}, height: {}, ratio: {}'.format(label_counts[right_lane_id], max_v, labels_im_br_height, max_v/labels_im_br_height))

            if max_v >= lane_bottom_tol*labels_im_br_height: # require the seed component to reach the very bottom of the image
                id_usable = True
                break

    roi_delta = 50
    roi_offset = 0
    while right_lane_id is None or label_counts[right_lane_id] < MIN_NumPx_RightLane or not id_usable:
        roi_offset += roi_delta
        if roi_offset > (labels_im.shape[1]*2)//3:
            print("FAILED")
            return img_rs, cc_im, None, None

        labels_im_bottom_right = labels_im[(labels_im.shape[0]*2)//3:, max(0,(labels_im.shape[1]*2)//3 - roi_offset):] # use bottom right area as seed for right limit line

        label_counts = np.bincount(labels_im_bottom_right.flatten()) # get the largest component in the bottom right area (with a min size, and that starts to the bottom of the image)

        print("WARNING: no seed (> {} px) for right lane found, increasing RoI to the left. current shape: {}, counts: {}".format(MIN_NumPx_RightLane, labels_im_bottom_right.shape, label_counts))

        if len(label_counts) > 1:
            #right_lane_id = np.argmax(label_counts[1:])+1 # exclude zero label (bg)
            sort_idx = np.argsort(label_counts[1:]) + 1 # ascending order

            for i in range(len(sort_idx)-1, -1, -1):
                right_lane_id = sort_idx[i]
                if label_counts[right_lane_id] < MIN_NumPx_RightLane:
                    break

                lane_idxs = np.where(labels_im_bottom_right == right_lane_id)

                max_v = lane_idxs[0].max()
                labels_im_br_height = labels_im_bottom_right.shape[0]

                print('current seed: count: {}, max_v: {}, height: {}, ratio: {}'.format(label_counts[right_lane_id], max_v, labels_im_br_height, max_v/labels_im_br_height))

                if max_v >= lane_bottom_tol*labels_im_br_height: # require the seed component to reach the very bottom of the image
                    id_usable = True
                    break

    print("chosing component with ID {} as right line reference".format(right_lane_id))

    labels_im_rl = np.where(labels_im == right_lane_id, right_lane_id, 0.)

    #The below requires `pip install opencv-contrib-python`
    thinned = cv2.ximgproc.thinning((labels_im_rl*255/right_lane_id).astype(np.uint8))

    #cc_im_rl = color_components(labels_im_rl)
    cc_im_rl = color_components(thinned.astype(np.float32))

    return img_rs, cc_im, cc_im_rl, thinned

def create_linestrip(id, positions, color, scale, marker_type=Marker.LINE_STRIP):
    marker = Marker()
    marker.header.frame_id = BASE_FRAME
    marker.header.stamp = rospy.Time.now()

    marker.ns = "camera_path_line"
    marker.id = id
    marker.type = marker_type 
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    for i in range(positions.shape[1]):
        pt = ROS_Point()
        pt.x = positions[0,i]
        pt.y = positions[1,i]
        pt.z = 0
        marker.points.append(pt)
        marker.colors.append(marker.color)

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    return marker

def create_marker(id, position, color, scale, marker_type=Marker.SPHERE):
    marker = Marker()
    marker.header.frame_id = BASE_FRAME
    marker.header.stamp = rospy.Time.now()

    marker.ns = "camera_path"
    marker.id = id
    marker.type = marker_type 
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = 0

    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]

    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]

    return marker

class Inferencer(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1.)

        caminfo_topic = '/freicar_4/d435/color/camera_info'
        rospy.loginfo("Wating for camera info message on topic: {}".format(caminfo_topic))

        self.caminfo_msg = rospy.wait_for_message(caminfo_topic, CameraInfo)
        rospy.loginfo("Received caminfo for frame {}\n{}".format(self.caminfo_msg.header.frame_id, self.caminfo_msg.K))

        self.T_base_cam_msg = None
        while self.T_base_cam_msg is None and not rospy.is_shutdown():
            try:
                self.T_base_cam_msg = self.tfBuffer.lookup_transform(BASE_FRAME, self.caminfo_msg.header.frame_id, rospy.Time())
            except Exception as e:
                rospy.logwarn(e)
                rospy.sleep(1.)

        if self.T_base_cam_msg is None:
            rospy.logerr("TF Lookup failed, aborting")
            exit(1)

        # transform to base_link:
        #- approx. Translation: [0.234, 0.024, 0.318]
        #- approx. Rotation: in Quaternion [-0.500, 0.500, -0.500, 0.500]
        self.T_base_cam = np.eye(4)
        self.T_base_cam[:3,:3] = R.from_quat([self.T_base_cam_msg.transform.rotation.x, self.T_base_cam_msg.transform.rotation.y, self.T_base_cam_msg.transform.rotation.z, self.T_base_cam_msg.transform.rotation.w]).as_matrix()
        self.T_base_cam[:3,3] = np.array([self.T_base_cam_msg.transform.translation.x, self.T_base_cam_msg.transform.translation.y, self.T_base_cam_msg.transform.translation.z])
        print("Transform camera to base_link:\n{}".format(self.T_base_cam))

        self.publisher_img_out = rospy.Publisher("/freicar_4/inference_image", Image, queue_size=1)
        self.publisher_cc_im = rospy.Publisher("/freicar_4/inference_image_processed_1", Image, queue_size=1)
        self.publisher_cc_im_rl = rospy.Publisher("/freicar_4/inference_image_processed_2", Image, queue_size=1)
        self.publisher_marker = rospy.Publisher("/freicar_4/camera_path", MarkerArray, queue_size=10)

        #self.publisher_da_out = rospy.Publisher("/freicar_4/inference_da", Image, queue_size=1)
        #self.publisher_ll_out = rospy.Publisher("/freicar_4/inference_ll", Image, queue_size=1)

        self.image_height = 360
        self.image_width = 640

        self.scale_x = self.image_width / self.caminfo_msg.width
        self.scale_y = self.image_height / self.caminfo_msg.height
        rospy.loginfo("detected scale x: {}, y: {} between camera image and network input. Rescaling K accordingly.".format(self.scale_x, self.scale_y))
        self.K_scaled = np.array(self.caminfo_msg.K).reshape(3,3).copy()
        self.K_scaled[0,:] *= self.scale_x
        self.K_scaled[1,:] *= self.scale_y
        print(self.K_scaled)
        print("Distortion: {}".format(self.caminfo_msg.D)) #FIXME: it seems that distortion is not part of rs_d435 factory calibration..

        self.image_sub = message_filters.Subscriber("/freicar_4/d435/color/image_raw/compressed", CompressedImage, queue_size=1, buff_size = 16777216)
        self.da_sub = message_filters.Subscriber("/freicar_4/inference_da", Image, queue_size=1, buff_size = 16777216)
        self.ll_sub = message_filters.Subscriber("/freicar_4/inference_ll", Image, queue_size=1, buff_size = 16777216)
        self.depth_sub = message_filters.Subscriber("/freicar_4/d435/aligned_depth_to_color/image_raw", Image, queue_size=1, buff_size = 16777216)

        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.da_sub, self.ll_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.image_callback)

    def infer(self, input_img, da_pred, ll_pred, depth, header):

        t1 = time.time()
        img_resized = cv2.resize(input_img, (self.image_width, self.image_height)) # resize to match network input size

        t2 = time.time()
        #inference skipped
        t3 = time.time()

        img_out, cc_im, cc_im_rl, rl_mask = postprocess(img_resized, da_pred, ll_pred)

        if rl_mask is None:
            rospy.logwarn("line extraction failed!")
            return img_out, cc_im, cc_im_rl, None

        t4 = time.time()
        # get pixel coordinates of right lane
        right_lane_vu = np.where(rl_mask>0.)
        print("right lane has {} pixels.".format(len(right_lane_vu[0])))

        depth_subsampled = subsample_depth(depth, int(round(1./self.scale_x)), int(round(1./self.scale_y)))

        print("subsampled depth shape: {}, mask shape: {}".format(depth_subsampled.shape, rl_mask.shape))

        right_lane_z = depth_subsampled[rl_mask>0.]
        print("retrieved {} depth values for right lane".format(len(right_lane_z)))

        ind_depth_valid = right_lane_z > 0.
        right_lane_v = right_lane_vu[0][ind_depth_valid]
        right_lane_u = right_lane_vu[1][ind_depth_valid]
        right_lane_z = right_lane_z[ind_depth_valid]
        print("retrieved {} VALID depth values for right lane".format(len(right_lane_z)))

        undistorted_points = cv2.undistortPoints(np.vstack([right_lane_u, right_lane_v]).astype(np.float32), self.K_scaled, self.caminfo_msg.D) # undistort before projection.

        #print("undistorted points: {}".format(undistorted_points.shape))
        #undistorted points are normalized
        cx = 0 #self.K_scaled[0,2] #rl_mask.shape[1] / 2.
        cy = 0 #self.K_scaled[1,2] #rl_mask.shape[0] / 2.
        fx = 1 #self.K_scaled[0,0] #600.
        fy = 1 #self.K_scaled[1,1] #600.

        right_lane_x = right_lane_z / fx * (undistorted_points[:,0,0] - cx) # right_lane_u
        right_lane_y = right_lane_z / fy * (undistorted_points[:,0,1] - cy) # right_lane_v´

        right_lane_points = np.vstack([right_lane_x, right_lane_y, right_lane_z])
        #print("right lane pointcloud shape: {}".format(right_lane_points.shape))

        right_lane_points_base = np.matmul(self.T_base_cam, np.vstack([right_lane_points,np.ones_like(right_lane_x)]))

        #subsample(e.g. 20cm res.)
        vox_res = 0.2
        xmin = right_lane_points_base[0,:].min()
        xmax = right_lane_points_base[0,:].max()
        ymin = right_lane_points_base[1,:].min()
        ymax = right_lane_points_base[1,:].max()

        vx = np.arange(xmin, xmax+vox_res, vox_res)
        vy = np.arange(ymin, ymax+vox_res, vox_res)
        vxofs,vyofs = np.meshgrid(vx, vy) #  ,indexing='ij' ?

        # create a KDTree datastructure for query
        tree = cKDTree(np.c_[right_lane_points_base[0,:].ravel(), right_lane_points_base[1,:].ravel()])

        # query of the one (k=1) closest point to each voxel center
        dd, ii = tree.query(np.c_[vxofs.ravel(), vyofs.ravel()], k=1)

        val_idx = np.unique(ii[dd < vox_res/2]) # use only points that are not farther away than half a voxel width, and every point is only used once
        print("{} points left after subsampling".format(len(val_idx)))
        x_sub = right_lane_points_base[0,:].ravel()[val_idx]
        y_sub = right_lane_points_base[1,:].ravel()[val_idx]

        min_idx = np.argmin(x_sub**2+y_sub**2) # start point is the point closest to current pos.
        start_dist = x_sub[min_idx]**2+y_sub[min_idx]**2

        #print("path start distance to car: {}m".format(start_dist))
        if start_dist > 2.5:
            rospy.logwarn("Line start too far from car: {}m. Not updating path!".format(start_dist))
            return img_out, cc_im, cc_im_rl, None

        points_list = np.vstack([x_sub, y_sub]).transpose().tolist()

        # order the points based on the known first point to form a continuous line.
        points_line = order_points(points_list, min_idx)
        xline, yline  = np.array(points_line).T

        dx = xline[1:] - xline[:-1]
        dy = yline[1:] - yline[:-1]

        nx = -dy
        ny = dx

        line = np.vstack([xline[:-1], yline[:-1]])
        normals = np.vstack([nx, ny])
        normals /= np.linalg.norm(normals, axis=0)

        path_offset = 0.23 #half the lane width. evtl. adapth to drive such that outer line is always visible.

        path = line + path_offset * normals

        #re-order to form smoot line:
        path_ordered = order_points(path.transpose().tolist(), 0)
        path_ordered  = np.array(path_ordered).T

        t5 = time.time()

        dpre = t2-t1
        dinf = t3-t2
        dpost = t4-t3
        dpath = t5-t4

        print("runtimes: preproc: {}ms; inf: {}ms; postproc: {}ms; calc path: {}ms".format(dpre*1000, dinf*1000, dpost*1000, dpath*1000))


        #verification: project path back to input image
        path_homog = np.vstack([path_ordered, np.zeros_like(path_ordered[0]), np.ones_like(path_ordered[0])])
        path_cam = np.matmul(np.linalg.inv(self.T_base_cam), path_homog)

        path_cam = path_cam[:3, path_cam[2] > 0]

        #path_u = fx * path_cam[0] / path_cam[2] + cx
        #path_v = fy * path_cam[1] / path_cam[2] + cy
        path_u = self.K_scaled[0,0] * path_cam[0] / path_cam[2] + self.K_scaled[0,2]
        path_v = self.K_scaled[1,1] * path_cam[1] / path_cam[2] + self.K_scaled[1,2]

        img_path = img_resized.copy()
        for path_idx in range(len(path_u)-1):
            img_path = cv2.line(img_path, (round(path_u[path_idx]), round(path_v[path_idx])), (round(path_u[path_idx+1]), round(path_v[path_idx+1])), (0, 127, 127), 3)

        #publish the path points as markers in baselink frame (use z=0)
        return img_path, cc_im, cc_im_rl, path_ordered

    def image_callback(self, msg, msg_da, msg_ll, msg_depth):

        #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

        da_pred = self.bridge.imgmsg_to_cv2(msg_da)
        ll_pred = self.bridge.imgmsg_to_cv2(msg_ll)

        depth_np = self.bridge.imgmsg_to_cv2(msg_depth) #encoding: 16UC1
        #print("depth dtype: {}".format(depth_np.dtype))
        depth_metric = depth_np.astype(np.float32) / DEPTH_SCALE

        img_out, cc_im, cc_im_rl, path = self.infer(cv_image, da_pred, ll_pred, depth_metric, msg.header)

        if path is not None:
            marker_array = MarkerArray()
            #for i in range(path.shape[1]):
            #    marker_array.markers.append(create_marker(i+1,[path[0,i], path[1,i] ],[0,1.,0,1.],[0.05,0.05,0.05]))

            line_marker = create_linestrip(0, path,[0,1.,0,1.],[0.05,0.05,0.05])
            marker_array.markers.append(line_marker)
            self.publisher_marker.publish(marker_array)

        if img_out is not None:
            img_out = self.bridge.cv2_to_imgmsg(img_out, encoding="bgr8")
            self.publisher_img_out.publish(img_out)

        if cc_im is not None:
            cc_im = self.bridge.cv2_to_imgmsg(cc_im, encoding="bgr8")
            self.publisher_cc_im.publish(cc_im)

        if cc_im_rl is not None:
            cc_im_rl = self.bridge.cv2_to_imgmsg(cc_im_rl, encoding="bgr8")
            self.publisher_cc_im_rl.publish(cc_im_rl)

    def destroy(self):
        pass #self.cuda_ctx.pop()


def main():
    rospy.init_node("inference")
    inferencer = Inferencer()
    rospy.loginfo("Lane Perception Node is running...")
    rospy.spin()

    inferencer.destroy()

if __name__ == '__main__':
    main()

