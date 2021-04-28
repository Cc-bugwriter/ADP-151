#include <tf_map/tf_map.h>

namespace tf_map
{
MapBroadcast::MapBroadcast(ros::NodeHandle& nh, ros::NodeHandle& private_nh) : nh(nh), p_nh(private_nh)
{
  carmaker_sub_ = nh.subscribe("/CMInfo",10,&MapBroadcast::CMCallback,this);
  waypoints_sub_ = nh.subscribe("/waypoint_loader_node/waypoints_array", 1, &MapBroadcast::WaypointsCallback, this);
  carmaker_pub_ = p_nh.advertise<visualization_msgs::Marker>("carmaker", 1, true);
  vechinfo_pub_ = p_nh.advertise<vaafo_msgs::CarInfo>("/VechInfo", 1, true);

  nh.getParam("vehicle_frame" , vehicle_frame);
  nh.getParam("origin_frame" , map_frame);
  nh.getParam("refer_line_num" , refer_line_num_);

  ROS_INFO("start tf map Node");
  //call visualzation

}

MapBroadcast::~MapBroadcast()
{
}

void MapBroadcast::CMCallback(const vaafo_msgs::CarInfo &cm_msg){

  vech_ = cm_msg;
  if(waypoint_called){

  //std::cout<<tf::getYaw(vech_.pose.orientation)<<", "<<vech_.pose.position.x<<", "<<vech_.pose.position.y<<", "<<vech_.twist.linear.x<<", "<<vech_.twist.linear.y<<", "<<vech_.acceleration.linear.x<<", "<<vech_.acceleration.linear.y<<std::endl;
 }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(vech_.pose.position.x, vech_.pose.position.y, vech_.pose.position.z) );
  tf::Quaternion q;
  double theta = tf::getYaw(vech_.pose.orientation);
  q.setRPY(0, 0, theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, vech_.header.stamp, map_frame, vehicle_frame));

  vechinfo_pub_.publish(vech_);
}

void MapBroadcast::WaypointsCallback(const vaafo_msgs::LaneArrayConstPtr &lane_array_msg){
  all_lines.lanes = lane_array_msg->lanes;
  if(!waypoint_called){
    waypoint_called = true;
  }

}

}  // end of namespace tf_map
