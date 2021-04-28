#ifndef waypoint_loader_H
#define waypoint_loader_H

#include <ros/ros.h>
#include <string>
#include <fstream>
#include <iostream>

#include "vaafo_msgs/Waypoint.h"
#include "vaafo_msgs/Lane.h"
#include "vaafo_msgs/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>

//include <actionlib/client/simple_action_client.h>

namespace waypoint_loader{
//typedef actionlib::SimpleActionClient<bla_msgs::GoToAction> BlaActionClient;
class WaypointLoader {

public:
  WaypointLoader(ros::NodeHandle& nh, ros::NodeHandle& private_nh );
    virtual ~WaypointLoader();

protected:

	//void blaCallback(const bla_msgs::Bla &bla_msg);

  void newWaypointLoader(std::string mapPath);
  void MapVisualization();
    	//void goToCmdFeedbackCB(const ias_robcom_msgs::GoToFeedbackConstPtr& feedback);
    	//void goToCmdDoneCb(const actionlib::SimpleClientGoalState& state,
                          //const ias_robcom_msgs::GoToResultConstPtr& result);

	ros::NodeHandle& nh;
	ros::NodeHandle& p_nh;
	
private:
  ros::Publisher waypoint_pub_;
  ros::Publisher marking_line_pub_;
  ros::Publisher reference_line_pub_;
        //ros::Subscriber sub_;
	//BlaActionClient* bla_action_client;


  std::string mapPath;
  std::string highwayPath;
  std::string line;

  vaafo_msgs::LaneArray lane_array_;
  visualization_msgs::Marker reference_lane_marker,marking_lane_marker;
  visualization_msgs::MarkerArray reference_lane_markerarray,marking_lane_markerarray;

};

} // end of namespace waypoint_loader

#endif // waypoint_loader_H
