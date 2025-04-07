#!/usr/bin/env python3

import rospy
import numpy
import math
from geometry_msgs.msg import Pose
from freicar_msgs.msg import ControlCommand
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class Controller(object):

    last_known_position_x = 0.00
    last_known_position_y = 0.00
    last_known_odometry_time = 0.00
    calculated_velocity = 0.00
    velocity_accuracy = 0.00
    throttle = 0.00
    prev_time = 0.00

    integral = 0.00
    i = 0.15
    p = 0.05
    d = 0.011
    prev_error = 0.00

    def __init__(self):

        rospy.init_node("Controller", anonymous=True)
        rospy.loginfo("Waiting for path on topic: /freicar_4/lookahead_point" )

        self.lookahead_point = rospy.Subscriber("/freicar_4/lookahead_point", Pose, self.lookahead_callback, queue_size=10)
        self.odometry = rospy.Subscriber("/freicar_4/t265/odom/sample", Odometry, self.odometry_callback, queue_size=10)
        self.control_cmd_pub = rospy.Publisher("/freicar_4/control", ControlCommand, queue_size=10)
        self.control_mode_pub = rospy.Publisher("/freicar_4/control_mode", Bool, queue_size=10)

        rospy.loginfo("Subscriber initialized")

    def lookahead_callback(self, msg):
        # rospy.loginfo("Received Pose")
        # rospy.loginfo("  Position -> x: %.2f, y: %.2f", msg.position.x, msg.position.y)
        point = [msg.position.x, msg.position.y]
        alpha = numpy.arctan2(point[1], point[0])
        # rospy.loginfo("Alpha: %.2f", alpha)
        delta = numpy.arctan((2*0.35*numpy.sin(alpha))/(numpy.sqrt(point[0]**2 + point[1]**2)))
        # rospy.loginfo("Delta radian: %.2f", delta)
        self.send_control_cmd(delta)

    def odometry_callback(self, msg):
        # rospy.loginfo("Odometry X: %.2f", msg.pose.pose.position.x)
        # rospy.loginfo("Odometry Y: %.2f", msg.pose.pose.position.y)
        current_time = msg.header.stamp
        if self.last_known_odometry_time is not 0.00:
            dt = abs((self.last_known_odometry_time - current_time).to_sec())
            dx = abs(self.last_known_position_x - msg.pose.pose.position.x)
            dy = abs(self.last_known_position_y - msg.pose.pose.position.y)
            self.calculated_velocity = numpy.sqrt(dx**2 + dy**2)/dt
            self.pid_step(0.05 - msg.twist.twist.linear.x, current_time)
            self.velocity_accuracy = 100 - abs((self.calculated_velocity / msg.twist.twist.linear.x) - 1) 
            # rospy.loginfo("Current Velocity: {:.2f}".format(self.calculated_velocity))
            # rospy.loginfo("Velocity Accuracy: {:.4f}".format(self.velocity_accuracy))
            # rospy.loginfo("Time since last message: {:.6f} seconds".format(dt))
            # rospy.loginfo("Last known position: X={:.2f}   y={:.2f}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
            self.last_known_position_x = msg.pose.pose.position.x
            self.last_known_position_y = msg.pose.pose.position.y

            

        self.last_known_odometry_time = current_time

    def pid_step(self, error, time):
        dt = abs(time.to_sec() - self.prev_time.to_sec())

        delta_e = (error - self.prev_error) / dt
        self.integral += error * dt 
        self.integral = min(self.integral, 1.0)

        self.throttle = self.p * error + self.i * self.integral + self.d * delta_e
        
        self.prev_time = time
        self.prev_error = error

    def send_control_cmd(self, delta):

        delta = delta / numpy.pi * 180
        # rospy.loginfo("Delta degree: %.2f", delta)
        # delta = 0.5
        command = ControlCommand()
        command.header.frame_id = "freicar_4"
        command.header.stamp = rospy.Time().now()
        if self.throttle >= 0.00 and self.throttle <= 0.15:
            command.throttle = min(self.throttle, 0.06)
        else:
            command.throttle = 0.00
        rospy.loginfo("Throttle: {:.4f}".format(self.throttle))
        command.steering = delta

        self.control_cmd_pub.publish(command)
        # rospy.loginfo("Command Sent")



    def run(self):
        self.prev_time = rospy.Time.now()
        rospy.spin()


def main():
    controller = Controller()
    rospy.loginfo("Controller Node is running...")
    controller.run()

if __name__ == '__main__':
    main()
