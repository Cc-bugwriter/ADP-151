#ifndef tf_map_H
#define tf_map_H

#include <ros/ros.h>
#include "vaafo_msgs/CarInfo.h"
#include "vaafo_msgs/LaneArray.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

namespace tf_map
{//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class MapBroadcast
{
public:
  MapBroadcast(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  virtual ~MapBroadcast();

protected:


  void WaypointsCallback(const vaafo_msgs::LaneArrayConstPtr &lane_array_msg);
  void CMCallback(const vaafo_msgs::CarInfo &cm_msg);
  void visualzation_carmaker();

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;

private:	
  ros::Publisher carmaker_pub_;
  ros::Publisher vechinfo_pub_;
  ros::Subscriber carmaker_sub_;
  ros::Subscriber waypoints_sub_;

  vaafo_msgs::LaneArray all_lines;
  bool waypoint_called;

  std::string vehicle_frame, map_frame;
  int refer_line_num_;
  vaafo_msgs::CarInfo vech_;

};

}  // end of namespace tf_map

#endif  // tf_map_H
