#include <ros/ros.h>
#include <waypoint_loader/waypoint_loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader_node");
    ROS_DEBUG("Starting waypoint_loader Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");

    waypoint_loader::WaypointLoader waypoint_loader(nh,p_nh);
    ros::spin();
    exit(0);
}
