#include <ros/ros.h>
#include <tf_map/tf_map.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_map_node");
  ROS_DEBUG("Starting tf_map Node");
  ros::NodeHandle nh("");
  ros::NodeHandle p_nh("~");
  tf_map::MapBroadcast map_broadcast(nh, p_nh);
  ros::spin();
  exit(EXIT_SUCCESS);
}
