#include <ros/ros.h>
#include <coordinate_transformation/coordinate_transformation.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "coordinate_transformation_node");
    ROS_DEBUG("Starting coordinate_transformation Node");
    ros::NodeHandle nh("");
    ros::NodeHandle p_nh("~");
    coordinate_transformation::CoordinateTransformation ct(nh,p_nh);
    ros::spin();
    exit(0);
}
