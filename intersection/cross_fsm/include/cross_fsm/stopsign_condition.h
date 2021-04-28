#ifndef STOPSIGN_CONDITION_H
#define STOPSIGN_CONDITION_H
#include "condition.h"


class StopSignCondition :public Condition
{
protected:
    // static observe area(in meter)
    static double boundary_x_near;
    static double boundary_x_far;
    static double boundary_y_near_static;
    static double boundary_y_far_static;
    // static observe area(in second)
    static double boundary_y_near_dynamic;
    static double boundary_y_far_dynamic;
    static double left_boundary_x_static;
    static double right_boundary_x_static;
    static double forward_boundary_x_static;


    static int stop_catch;

    struct Security {
        bool right;
        bool left;
        bool forwards;
    }security;

public:
    StopSignCondition(queryVariable&);
    bool queue_check();
    int stop_times_counter();
    Security security_check();
    bool check_right(vaafo_msgs::DetectedObject);
    bool check_left(vaafo_msgs::DetectedObject);
    bool check_forward(vaafo_msgs::DetectedObject);

    virtual int motivation_detection();
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
