#ifndef QUERY_VARIABLE_H
#define QUERY_VARIABLE_H
#include <ros/ros.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/DetectedObject.h>
#include <vaafo_msgs/DetectedObjectArray.h>
#include <vaafo_msgs/SensorObjectList.h>
#include <vaafo_msgs/SensorObjectInfo.h>
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/tf.h>
#include <math.h>


class queryVariable
{
public:
    queryVariable();
    ~queryVariable(){

    }
    void callback1(const vaafo_msgs::CarInfo msg);
    void callback2(const vaafo_msgs::DetectedObjectArray msg);
    // get the position and speed of the nearby five cars: f,fl,fr,bl,br (in order)
    void callback3(const vaafo_msgs::SensorObjectList msg);


    double* computeD(vaafo_msgs::DetectedObject object);
    // required variables for conditions
    int ego_lane_id;
    vaafo_msgs::CarInfo egoCar;
    // the postion and velocity of egoCar after transformation
    vaafo_msgs::CarInfo egoCar_new;
    // the target position and velocity of egoCar:
    vaafo_msgs::CarInfo egoCar_target;

    // read the msg of the objects around the ego car
    std::vector<vaafo_msgs::DetectedObject> objects;
    std::vector<vaafo_msgs::SensorObjectInfo> lidar_objects;

    ros::NodeHandle nh;
    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;


    vaafo_msgs::DetectedObject nearObjects[5];
    vaafo_msgs::SensorObjectInfo otherObjects[5];

    void coord_frenet();
    void get_otherObjects();
    void set_target_ego();


};
#endif // QUERY_VARIABLE_H


