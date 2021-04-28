#ifndef coordinate_transformation_H
#define coordinate_transformation_H

#include <ros/ros.h>
#include "vaafo_msgs/CarInfo.h"
#include "vaafo_msgs/SensorObjectList.h"
#include "vaafo_msgs/DetectedObjectArray.h"
#include "vaafo_msgs/LaneArray.h"
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace coordinate_transformation{
typedef message_filters::sync_policies::ApproximateTime<vaafo_msgs::SensorObjectList, vaafo_msgs::SensorObjectList,vaafo_msgs::CarInfo> SensorSyncPolicy;
class CoordinateTransformation {

public:
  CoordinateTransformation(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~CoordinateTransformation();

protected:

  void Sychron_Callback(const vaafo_msgs::SensorObjectListConstPtr &radar_msg,const vaafo_msgs::SensorObjectListConstPtr &lidar_msg,const vaafo_msgs::CarInfoConstPtr &cm_msg);
  tf::StampedTransform updateNecessaryTransform(const std_msgs::Header& header);
  geometry_msgs::Twist getTransformedTwist(const geometry_msgs::Twist& in_twist,const tf::StampedTransform& tf_stamp);
  geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose,const tf::StampedTransform& tf_stamp);
  void convertsensorobjinfo2detectedobj(const vaafo_msgs::SensorObjectInfo& obj_info);
  void WaypointsCallback(const vaafo_msgs::LaneArray &lane_array_msg);
  void getsd(vaafo_msgs::DetectedObject& obj_);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
  tf::TransformListener tf_listener_;
	
private:
  ros::Publisher obj_pub_;
  message_filters::Subscriber<vaafo_msgs::SensorObjectList>* radar_sub_;
  message_filters::Subscriber<vaafo_msgs::SensorObjectList>* lidar_sub_;
  message_filters::Subscriber<vaafo_msgs::CarInfo>* carmaker_sub_;
  ros::Subscriber waypoint_sub_;

  message_filters::Synchronizer<SensorSyncPolicy>* sync_final;

  std::string vehicle_frame, map_frame;
  double radar_offset_x,radar_offset_y,radar_offset_z,lidar_offset_x,lidar_offset_y,lidar_offset_z;
  Eigen::Vector3d radar_offset,lidar_offset;
  tf::StampedTransform radar2map, lidar2map;
  Eigen::Matrix3d omega_;

  vaafo_msgs::LaneArray all_lines;
  bool waypoint_called = false;
  int refer_line_num_;

};

} // end of namespace coordinate_transformation

#endif // coordinate_transformation_H
