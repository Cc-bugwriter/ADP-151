#ifndef TRAFFICLIGHT_CONDITION_H
#define TRAFFICLIGHT_CONDITION_H
#include "condition.h"



class TrafficLightCondition :public Condition
{
protected:
    static int traffic_cache;
    static double crosswalk_pose_x_near;
    static double crosswalk_pose_x_far;
    static double slow_approach;

public:
    TrafficLightCondition(queryVariable&);
    virtual int yellow_recognition();
    virtual int motivation_detection() ;
    virtual bool collision_check();
    virtual int area_recognition();

    virtual bool isBufferToKeepLane();
    virtual bool isBufferToPrepLeft();
    virtual bool isBufferToPrepRight();
    virtual bool isBufferToStop();

    virtual bool isKeepLaneReflexive();
    virtual bool isKeepLaneToPrepLeft();
    virtual bool isKeepLaneToPrepRight();
    virtual bool isKeepLaneToStop();
    virtual bool isKeepLaneToBuffer();

    virtual bool isStopReflexive();
    virtual bool isStopToKeepLane();
    virtual bool isStopToPrepLeft();
    virtual bool isStopToPrepRight();

    virtual bool isPrepLeftToKeepLane();
    virtual bool isPrepLeftToStop();
    virtual bool isPrepLeftToTurnLeft();

    virtual bool isPrepRightToKeepLane();
    virtual bool isPrepRightToStop();
    virtual bool isPrepRightToTurnRight();

    virtual bool isTurnLeftToKeepLane();
    virtual bool isTurnLeftToStop();

    virtual bool isTurnRightToKeepLane();
    virtual bool isTurnRightToStop();


};
#endif
