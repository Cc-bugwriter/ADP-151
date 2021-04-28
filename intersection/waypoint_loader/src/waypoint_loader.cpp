#include <waypoint_loader/waypoint_loader.h>
namespace waypoint_loader{

WaypointLoader::WaypointLoader(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
	: nh(nh)
	, p_nh(private_nh)
{
    nh.getParam("waypoint_loader_node/mapFileName" , mapPath);
    //nh.getParam("waypoint_loader_node/Autobahn", highwayPath);

    waypoint_pub_ = p_nh.advertise<vaafo_msgs::LaneArray>("waypoints_array", 1, true);
    marking_line_pub_ = p_nh.advertise<visualization_msgs::MarkerArray>("marking_lines_marker", 1, true);
    reference_line_pub_ = p_nh.advertise<visualization_msgs::MarkerArray>("reference_lines_marker", 1, true);

   newWaypointLoader(mapPath);
   //newWaypointLoader(highwayPath);
}


WaypointLoader::~WaypointLoader()
{}


void WaypointLoader::newWaypointLoader(std::string map_file_){


  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  int lane_num = 0;
  reference_lane_markerarray.markers.clear();
  marking_lane_markerarray.markers.clear();

  MapVisualization();

  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    vaafo_msgs::Lane lane_;
    std::string temp_result;
    marking_lane_marker.points.clear();
    reference_lane_marker.points.clear();

    while (getline(iss,temp_result,' ')) {
        std::istringstream ss(temp_result);
        std::string x,y,s;
        vaafo_msgs::Waypoint lane_marking_waypoints_;
        geometry_msgs::Point point;


        getline(ss,x,',');
        getline(ss,y,',');
        if (getline(ss,s,',')){lane_marking_waypoints_.s_d.point.x = atof(s.c_str());}
        point.x = atof(x.c_str());
        point.y = atof(y.c_str());
        point.z = 0;

        if(lane_num%2==0){
           marking_lane_marker.points.push_back(point);
        }else {
           reference_lane_marker.points.push_back(point);
        }

        lane_marking_waypoints_.pose.pose.position = point;
        lane_.waypoints.push_back(lane_marking_waypoints_); //all lines are published

    }


   lane_array_.lanes.push_back(lane_);

   //otherweise there is empty points in the array
 if(lane_num < 9){
   if(lane_num%2==0){
     marking_lane_marker.id = lane_num; //make sure every points have unique id
     marking_lane_markerarray.markers.push_back(marking_lane_marker);
   }else {
     reference_lane_marker.id = lane_num;
     reference_lane_markerarray.markers.push_back(reference_lane_marker);
   }
 }else {
   if(lane_num%2==1){
     marking_lane_marker.id = lane_num; //make sure every points have unique id
     marking_lane_marker.points = reference_lane_marker.points;
     marking_lane_markerarray.markers.push_back(marking_lane_marker);
   }else {
     reference_lane_marker.id = lane_num;
     reference_lane_marker.points = marking_lane_marker.points;
     reference_lane_markerarray.markers.push_back(reference_lane_marker);
   }
 }


   lane_num++;
 }

  std::cout <<"Load map successfully"<<std::endl;
  lane_array_.header.frame_id = "map";
  lane_array_.header.stamp = ros::Time::now();
  waypoint_pub_.publish(lane_array_);
  marking_line_pub_.publish(marking_lane_markerarray);
  reference_line_pub_.publish(reference_lane_markerarray);
}

void WaypointLoader::MapVisualization(){

  reference_lane_marker.header.frame_id = marking_lane_marker.header.frame_id = "map";
  reference_lane_marker.header.stamp = marking_lane_marker.header.stamp = ros::Time::now();
  marking_lane_marker.ns = "laneMarking";
  reference_lane_marker.ns = "referenceMarking";
  reference_lane_marker.type = marking_lane_marker.type = visualization_msgs::Marker::POINTS;
  reference_lane_marker.action = marking_lane_marker.action = visualization_msgs::Marker::ADD;
  marking_lane_marker.scale.x = 0.25;
  marking_lane_marker.scale.y = 1;
  reference_lane_marker.scale.x = 0.05;
  reference_lane_marker.scale.y = 1;
  marking_lane_marker.color.r = 1;
  marking_lane_marker.color.g = 1;
  marking_lane_marker.color.b = 0;
  marking_lane_marker.color.a = 1;
  reference_lane_marker.color.r = 1;
  reference_lane_marker.color.g = 1;
  reference_lane_marker.color.b = 1;
  reference_lane_marker.color.a = 0.5;
  marking_lane_marker.frame_locked = false;
  reference_lane_marker.frame_locked = false;
}


} // end of namespace waypoint_loader
