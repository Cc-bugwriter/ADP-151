#ifndef CONDITION_H
#define CONDITION_H
#include <ros/ros.h>
#include "query_variable.h"
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/DetectedObject.h>
#include <vaafo_msgs/DetectedObjectArray.h>
#include <vaafo_msgs/TrafficLightResult.h>
#include <vaafo_msgs/Lane.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <tf/tf.h>

using namespace std;

class Condition
{
public:
    Condition(queryVariable&);
    virtual ~Condition();

    //// get the position and speed of the nearby five cars: f,fl,fr,bl,br (in order)
    //// required variables for conditions
    //int ego_lane_id;
    //NearObject egoCar;
    //std::vector<vaafo_msgs::DetectedObject> objects;
    //NearObject f_Object;
    //NearObject fl_Object;
    //NearObject fr_Object;
    //NearObject bl_Object;
    //NearObject br_Object;

    virtual int motivation_detection() = 0;
    virtual bool collision_check() = 0;
    virtual int area_recognition() = 0;

    virtual bool isBufferToKeepLane() = 0;
    virtual bool isBufferToPrepLeft() = 0;
    virtual bool isBufferToPrepRight() = 0;
    virtual bool isBufferToStop() = 0;

    virtual bool isKeepLaneReflexive() = 0;
    virtual bool isKeepLaneToPrepLeft() = 0;
    virtual bool isKeepLaneToPrepRight() = 0;
    virtual bool isKeepLaneToStop() = 0;
    virtual bool isKeepLaneToBuffer() = 0;

    virtual bool isStopReflexive() = 0;
    virtual bool isStopToKeepLane() = 0;
    virtual bool isStopToPrepLeft() = 0;
    virtual bool isStopToPrepRight() = 0;

    virtual bool isPrepLeftToKeepLane() = 0;
    virtual bool isPrepLeftToStop() = 0;
    virtual bool isPrepLeftToTurnLeft() = 0;

    virtual bool isPrepRightToKeepLane() = 0;
    virtual bool isPrepRightToStop() = 0;
    virtual bool isPrepRightToTurnRight() = 0;

    virtual bool isTurnLeftToKeepLane() = 0;
    virtual bool isTurnLeftToStop() = 0;

    virtual bool isTurnRightToKeepLane() = 0;
    virtual bool isTurnRightToStop() = 0;



protected:
    queryVariable &queryVar;
    // collision check required parameters
    static double acc_brake_max;            // maximal deceleration m / s ^ 2
    static double acc_brake_soft_max;       // maximal soft deceleration m / s ^ 2
    static double acc_brake_soft_min;       // minimal soft deceleration m / s ^ 2
    static double t_refresh;                // refresh time interval, s
    static double emb_min;                  // minimal emergency brake distance in low speed state, m
    static double emb_brake_proportion;     // emergency braking proportion in collision chech, [0, 1]

    // static line(in meter)
    static double crosswalk_pose_x_near;
    static double crosswalk_pose_x_far;
    static double slow_approach;

    // the mindest braking distance between the egocar and the front car, m
    double const s_emb_min= 125.0 / 9.0 * 1.2;
    const double PI = 3.141592653589793;
    
    //  coordinate offset
    static double radar_offset;
    static double carrosserie;
};
#endif












