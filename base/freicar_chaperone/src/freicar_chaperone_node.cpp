#include <Chaperone.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "freicar_chaperone");
    ros::NodeHandle nh("~");

    Chaperone chaperone(nh);
    chaperone.run();

    return 0;
}
