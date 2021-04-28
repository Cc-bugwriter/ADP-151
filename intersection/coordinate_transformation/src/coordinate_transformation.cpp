#include <coordinate_transformation/coordinate_transformation.h>

namespace coordinate_transformation{

CoordinateTransformation::CoordinateTransformation(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)
{
   obj_pub_ = p_nh.advertise<vaafo_msgs::DetectedObjectArray>("/finalObjList/map/objects", 1, true);

   radar_sub_ = new message_filters::Subscriber<vaafo_msgs::SensorObjectList> (nh, "/radar_objlist", 1);
   lidar_sub_ = new message_filters::Subscriber<vaafo_msgs::SensorObjectList> (nh, "/lidar_objlist", 1);
   carmaker_sub_ = new message_filters::Subscriber<vaafo_msgs::CarInfo> (nh, "/VechInfo", 1);

   sync_final = new message_filters::Synchronizer<SensorSyncPolicy> (SensorSyncPolicy(10), *radar_sub_, *lidar_sub_,*carmaker_sub_);
   sync_final->registerCallback(boost::bind(&CoordinateTransformation::Sychron_Callback, this, _1, _2, _3));

   waypoint_sub_ = nh.subscribe("/waypoint_loader_node/waypoints_array", 1, &CoordinateTransformation::WaypointsCallback, this);

   nh.getParam("vehicle_frame" , vehicle_frame);
   nh.getParam("origin_frame" , map_frame);
   nh.getParam("refer_line_num" , refer_line_num_);

   //define offset
   radar_offset << 3.53,0.0,0.103;
   lidar_offset << 1.23,0.0,1.503;
   ROS_INFO("start fusion Node");
}


CoordinateTransformation::~CoordinateTransformation()
{}

tf::StampedTransform CoordinateTransformation::updateNecessaryTransform(const std_msgs::Header& header){
  tf::StampedTransform sensor2baselink;
  try
  {
      tf_listener_.waitForTransform(header.frame_id, map_frame, header.stamp, ros::Duration(1.0));
      tf_listener_.lookupTransform(map_frame, header.frame_id, header.stamp, sensor2baselink);
  }
  catch (tf::TransformException ex)
  {
      ROS_ERROR("%s", ex.what());
  }
  return sensor2baselink;

}

geometry_msgs::Pose CoordinateTransformation::getTransformedPose(const geometry_msgs::Pose& in_pose,const tf::StampedTransform& tf_stamp){
  tf::Transform transform;
  geometry_msgs::PoseStamped out_pose;
  transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
  transform.setRotation(
              tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
  tf::poseTFToMsg(tf_stamp * transform, out_pose.pose);
  return out_pose.pose;

}


geometry_msgs::Twist CoordinateTransformation::getTransformedTwist(const geometry_msgs::Twist& in_twist,const tf::StampedTransform& tf_stamp){
  tf::Vector3 end;
  geometry_msgs::TwistStamped out_twist;
  tf::Vector3 origin = tf::Vector3(0,0,0);
  tf::vector3MsgToTF(in_twist.linear,end);
  tf::Vector3 output  = (tf_stamp * end) - (tf_stamp * origin);
  tf::vector3TFToMsg(output,out_twist.twist.linear);
  return out_twist.twist;

}

void CoordinateTransformation::WaypointsCallback(const vaafo_msgs::LaneArray &lane_array_msg){
  all_lines = lane_array_msg;
  if (all_lines.lanes.size()>0 && !waypoint_called){
    waypoint_called = true;
    //kd.BuildKDTree(all_lines.lanes.at(refer_line_num_));
  }
  else if(!waypoint_called){
    std::cout<<"map is not called yet"<<std::endl;
  }

}

void CoordinateTransformation::getsd(vaafo_msgs::DetectedObject& obj_){
/*  
if(waypoint_called){
  vaafo_msgs::Waypoint input,output;
  double dist;
  input.pose.pose = obj_.pose;
  kd.FindNearestPoint(input,output,dist);

  std::vector<double> ego_sd = mpc_planer_helpers::MpcPlanerHelpers::getFrenet(obj_.pose, output.inx, all_lines.lanes.at(refer_line_num_));
  obj_.lane_id = mpc_planer_helpers::MpcPlanerHelpers::getLane(ego_sd.at(1),refer_line_num_);
  obj_.s_d.x = ego_sd.at(0);
  obj_.s_d.y = ego_sd.at(1);

  //velocity along road and prependicular to road
  double v_ = hypot(obj_.velocity.linear.x,obj_.velocity.linear.y);
  obj_.velocity.angular.x = v_ * cos(ego_sd.at(2));
  obj_.velocity.angular.y = v_ * sin(ego_sd.at(2));

 }
*/

}

void CoordinateTransformation::Sychron_Callback(const vaafo_msgs::SensorObjectListConstPtr &radar,const vaafo_msgs::SensorObjectListConstPtr &lidar,const vaafo_msgs::CarInfoConstPtr &vech){
  radar2map = updateNecessaryTransform(radar->header);
  lidar2map = updateNecessaryTransform(lidar->header);
  if (!radar2map.getOrigin() || !lidar2map.getOrigin())
  {
      ROS_INFO("Could not find coordiante transformation radar/lidar to map");
      return;
  }

  omega_ << 0,-vech->twist.angular.z,vech->twist.angular.y,
            vech->twist.angular.z,0,-vech->twist.angular.x,
            -vech->twist.angular.y,vech->twist.angular.x,0;

  vaafo_msgs::DetectedObjectArray _radar_objlist_new;
  vaafo_msgs::DetectedObjectArray _lidar_objlist_new;
  vaafo_msgs::DetectedObjectArray _objlist_new;

  //transform radar objects
  for (unsigned int i=0;i<radar->objects.size();i++) {
    geometry_msgs::Twist _radar_twist_base_link,_radar_twist_map;
    geometry_msgs::Twist _radar_acc_base_link,_radar_acc_map;
    vaafo_msgs::DetectedObject _radar_out;

    //transform pose
    _radar_out.pose = getTransformedPose(radar->objects.at(i).pose,radar2map);
    _radar_out.pose_reliable = true;

    //transform velocity
    Eigen::Vector3d radar_pose(radar->objects.at(i).pose.position.x,radar->objects.at(i).pose.position.y,radar->objects.at(i).pose.position.z);
    Eigen::Vector3d _pose_offset = radar_pose + radar_offset;
    Eigen::Vector3d _v_add = omega_*_pose_offset;
    _radar_twist_base_link.linear.x = radar->objects.at(i).velocity.linear.x + _v_add[0];
    _radar_twist_base_link.linear.y = radar->objects.at(i).velocity.linear.y + _v_add[1];
    _radar_twist_base_link.linear.z = radar->objects.at(i).velocity.linear.z + _v_add[2];

    _radar_twist_map = getTransformedTwist(_radar_twist_base_link,radar2map);
    _radar_out.velocity.linear.x = _radar_twist_map.linear.x + vech->twist.linear.x; //vech velocity is positive
    _radar_out.velocity.linear.y = _radar_twist_map.linear.y + vech->twist.linear.y;
    _radar_out.velocity.linear.z = _radar_twist_map.linear.z + vech->twist.linear.z;
    _radar_out.velocity_reliable = true;

    //transform acceleration
    Eigen::Vector3d radar_acc(radar->objects.at(i).velocity.linear.x,radar->objects.at(i).velocity.linear.y,radar->objects.at(i).velocity.linear.z);
    Eigen::Vector3d _acc_add = omega_*radar_acc;
    _radar_acc_base_link.linear.x = radar->objects.at(i).acceleration.linear.x + 2*_acc_add[0];
    _radar_acc_base_link.linear.y = radar->objects.at(i).acceleration.linear.y + 2*_acc_add[1];
    _radar_acc_base_link.linear.z = radar->objects.at(i).acceleration.linear.z + 2*_acc_add[2];

    _radar_acc_map = getTransformedTwist(_radar_acc_base_link,radar2map);
    //acceleration from object is already global
    _radar_out.acceleration.linear.x = radar->objects.at(i).acceleration.linear.x;//_radar_acc_map.linear.x + vech->acceleration.linear.x;
    _radar_out.acceleration.linear.y = radar->objects.at(i).acceleration.linear.y;//_radar_acc_map.linear.y + vech->acceleration.linear.y;
    _radar_out.acceleration.linear.z = radar->objects.at(i).acceleration.linear.z;//_radar_acc_map.linear.z + vech->acceleration.linear.z;
    _radar_out.acceleration_reliable = true;

    //other properties
    _radar_out.id = radar->objects.at(i).traffic_id;
    _radar_out.dimensions.x = radar->objects.at(i).dimension.x;
    _radar_out.dimensions.y = radar->objects.at(i).dimension.y;
    _radar_out.dimensions.z = radar->objects.at(i).dimension.z;

    //calculate s and d value
    getsd(_radar_out);

   _radar_objlist_new.objects.push_back(_radar_out);
  }

  //transform lidar objects

  for (unsigned int i=0;i<lidar->objects.size();i++) {
    geometry_msgs::Twist _lidar_twist_base_link,_lidar_twist_map;
    geometry_msgs::Twist _lidar_acc_base_link,_lidar_acc_map;
    vaafo_msgs::DetectedObject _lidar_out;

    //transform pose
    _lidar_out.pose = getTransformedPose(lidar->objects.at(i).pose,lidar2map);
    _lidar_out.pose_reliable = true;

    //transform velocity
    Eigen::Vector3d lidar_pose(lidar->objects.at(i).pose.position.x,lidar->objects.at(i).pose.position.y,lidar->objects.at(i).pose.position.z);
    Eigen::Vector3d _pose_offset = lidar_pose + lidar_offset;
    Eigen::Vector3d _v_add = omega_*_pose_offset;
    _lidar_twist_base_link.linear.x = lidar->objects.at(i).velocity.linear.x + _v_add[0];
    _lidar_twist_base_link.linear.y = lidar->objects.at(i).velocity.linear.y + _v_add[1];
    _lidar_twist_base_link.linear.z = lidar->objects.at(i).velocity.linear.z + _v_add[2];

    _lidar_twist_map = getTransformedTwist(_lidar_twist_base_link,lidar2map);
    _lidar_out.velocity.linear.x = _lidar_twist_map.linear.x + vech->twist.linear.x;
    _lidar_out.velocity.linear.y = _lidar_twist_map.linear.y + vech->twist.linear.y;
    _lidar_out.velocity.linear.z = _lidar_twist_map.linear.z + vech->twist.linear.z;
    _lidar_out.velocity_reliable = true;

    //transform acceleration
    Eigen::Vector3d lidar_acc(lidar->objects.at(i).velocity.linear.x,lidar->objects.at(i).velocity.linear.y,lidar->objects.at(i).velocity.linear.z);
    Eigen::Vector3d _acc_add = omega_*lidar_acc;
    _lidar_acc_base_link.linear.x = lidar->objects.at(i).acceleration.linear.x + 2*_acc_add[0];
    _lidar_acc_base_link.linear.y = lidar->objects.at(i).acceleration.linear.y + 2*_acc_add[1];
    _lidar_acc_base_link.linear.z = lidar->objects.at(i).acceleration.linear.z + 2*_acc_add[2];

    _lidar_acc_map = getTransformedTwist(_lidar_acc_base_link,lidar2map);
    _lidar_out.acceleration.linear.x = lidar->objects.at(i).acceleration.linear.x; //+ vech->acceleration.linear.x;
    _lidar_out.acceleration.linear.y = lidar->objects.at(i).acceleration.linear.y; //+ vech->acceleration.linear.y;
    _lidar_out.acceleration.linear.z = lidar->objects.at(i).acceleration.linear.z; //+ vech->acceleration.linear.z;
    _lidar_out.acceleration_reliable = true;

    //other properties
    _lidar_out.id = lidar->objects.at(i).traffic_id;
    _lidar_out.dimensions.x = lidar->objects.at(i).dimension.x;
    _lidar_out.dimensions.y = lidar->objects.at(i).dimension.y;
    _lidar_out.dimensions.z = lidar->objects.at(i).dimension.z;

    //calculate s and d value
    getsd(_lidar_out);

   _lidar_objlist_new.objects.push_back(_lidar_out);
  }

  //fusion
  if (!_radar_objlist_new.objects.empty() && _lidar_objlist_new.objects.empty())
       _objlist_new = _radar_objlist_new;
  else if (_radar_objlist_new.objects.empty() && !_lidar_objlist_new.objects.empty()) {
      _objlist_new = _lidar_objlist_new;
  }else if (!_radar_objlist_new.objects.empty() && !_lidar_objlist_new.objects.empty()) {
     _objlist_new = _radar_objlist_new;

     for (unsigned int i=0;i<_lidar_objlist_new.objects.size();i++) {
       bool _repeat = false;
       for (unsigned int j=0;j<_radar_objlist_new.objects.size();j++) {
           if (_lidar_objlist_new.objects.at(i).id == _radar_objlist_new.objects.at(j).id){
             _repeat = true;
            break;}
       }
       if(!_repeat){
        _objlist_new.objects.push_back(_lidar_objlist_new.objects.at(i));
       }

     }

  }

  _objlist_new.header.frame_id = map_frame;
  _objlist_new.header.stamp = ros::Time::now();
  obj_pub_.publish(_objlist_new);

}



} // end of namespace coordinate_transformation
