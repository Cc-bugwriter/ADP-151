#ifndef PUB_CLASS_H
#define PUB_CLASS_H
#include "ros/ros.h"
#include <straight_fsm/vehicle.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/State.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include "query_variable.h"

class pub{
public:
   pub(ros::NodeHandle&);

   void pubState(const fsm_state*);
   geometry_msgs::Twist goalacceleration();

private:
   ros::Publisher chatter_pubstate;//set the publisher as a member

   double const t_cruise = 1.5;
};

#endif // PUB_CLASS_H
