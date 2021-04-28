#ifndef PUB_CLASS_H
#define PUB_CLASS_H
#include "ros/ros.h"
#include <cross_fsm/vehicle.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/State.h>
#include <cross_fsm/vehicle.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include "query_variable.h"
#include <fstream>
#include <sstream>

class pub{
public:
   pub(ros::NodeHandle&);

   void pubState(const fsm_state*, const queryVariable&);
   geometry_msgs::Pose goalposition(const fsm_state*, const queryVariable&);
   geometry_msgs::Point goalfrenet(const fsm_state*, const queryVariable&);
   geometry_msgs::Twist goalvelocity(const fsm_state*, const queryVariable&);
   geometry_msgs::Twist goalacceleration();

private:
   ros::Publisher chatter_pubstate;//set the publisher as a member
   double const t_cruise = 2.4;


/*
 * ------------------------------------------additional block------------------------------------
 *  load the reference line from map data for frenet transformation
 */
   void readMap(vector<double> &x_cord_, vector<double> &y_cord_) {
       //read waypoints from txt file
//        std::ifstream infile("/home/cui/catkin_WS/cross_fsm/src/trajectoryPlanner/path_file/interval40cm_right.txt");
       std::ifstream infile("/home/cui/catkin_WS/cross_fsm/src/trajectoryPlanner/path_file/interval40cm_straight.txt");
       string line;

       while (getline(infile, line))
       {
           istringstream iss(line);
           double a, b;

           if (!(iss >> a >> b)) break;// error

           x_cord_.push_back(a);
           y_cord_.push_back(b);
       }

   }
   std::vector<double> frenet_s;
   void calcS(std::vector<double>&);

   vector<double> x_cord;
   vector<double> y_cord;
};

#endif // PUB_CLASS_H
