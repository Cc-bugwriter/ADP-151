#include "cross_fsm/trafficlight_condition.h"
#include <cmath>

using namespace std;

// initialize protected parameter
int TrafficLightCondition::traffic_cache = 0;   // cache the previous traffic light id

TrafficLightCondition::TrafficLightCondition(queryVariable& qV): Condition(qV){}

// static line(in meter)
double TrafficLightCondition::crosswalk_pose_x_near = -1;   // crosswalk close to intersection
double TrafficLightCondition::crosswalk_pose_x_far = 2;     // crosswalk far away to intersection
double TrafficLightCondition::slow_approach = 7;            // stop area threshold for low veloctiy situation

/*
 *  detect the moving motivation of ego car in next intersection
 *  motivation:
 *      0:  forwards
 *      1:  turn left
 *      2:  turn right
 */
int TrafficLightCondition::motivation_detection() {

    return 0;
}

/*
 *  collision prediction model
 *  return: ture <-> further collision risk
 */
bool TrafficLightCondition::collision_check(){
    // check whether a collision appears in front of EgoCar

    // transform entity position and velocity in vehicle coordinate
    double entity_position_x = queryVar.RelativPositionX(queryVar.dec_Object.pose.position) - radar_offset; // collision distance
    double entity_position_y = queryVar.RelativPositionY(queryVar.dec_Object.pose.position);
    double entity_velocity_x = queryVar.RelativVelocityX(queryVar.dec_Object.velocity);
    double entity_velocity_y = queryVar.RelativVelocityY(queryVar.dec_Object.velocity);

    // define the check area
    bool Collision;

    // extent soft breaking (same as below)
    double acc_brake_soft = acc_brake_soft_max;
    if (queryVar.EgoVelocityX() <= 25.0 / 3.0)
    {
        // in case of slower than 30km / h
        acc_brake_soft = acc_brake_soft_max;
    }
    else if (queryVar.EgoVelocityX() >= 125.0 / 9.0)
    {
        // in case of faster than 50km / h
        acc_brake_soft = acc_brake_soft_min;
    }
    else
        // intermediate velocity phase[30km / h, 50km / h]
        acc_brake_soft = (acc_brake_soft_max + acc_brake_soft_min) / 2.0 - (acc_brake_soft_max - acc_brake_soft_min) / 2.0 * sin(PI / (125.0 / 9.0 - 25.0 / 3.0) * queryVar.egoCar.twist.linear.x);

    double collision_boundary_x = pow(entity_velocity_x,2)*(emb_brake_proportion/acc_brake_max + (1 - emb_brake_proportion)/acc_brake_soft) +
            queryVar.EgoVelocityX()*t_refresh + s_emb_min;
    double collision_boundary_y = 1.5;

    if(entity_velocity_x <0){
        if ((0< entity_position_x && entity_position_x<collision_boundary_x)&& (abs(entity_position_y)<collision_boundary_y)){
            Collision = true;
        }else {
            if (entity_velocity_y==0.0){
                Collision=false;
            }else {
                if(entity_position_x>0 && abs(entity_position_y)>1.5){
                    // traffic participant on both sides (right and left)
                    double t_cut_in = abs(entity_position_y)/entity_velocity_y;
                    // predict the obstacle position after cut in
                    double prediction_pose_x = entity_position_x + t_cut_in*entity_velocity_x;

                    if(0< prediction_pose_x &&
                            prediction_pose_x < collision_boundary_x){
                        Collision = true;
                    }else {
                        Collision = false;
                    }
                }else {
                    Collision=false;
                }
            }
        }
    }else {
        Collision=false;
    }

    return Collision;
}

//msg from carmaker identifies already two yellow light states, so this func is redundant in condition.
/*
 *  Recognize detected TrafficLight of yellow bulb
 *  @ param traffic : TrafficLightResult(), actual traffic state
 *  @ return yellow_station : int
 *      yellow_station = 0 : Green To Red
 *      yellow_station = 1 : Red To Green
 */
int TrafficLightCondition::yellow_recognition() {
    int yellow_station = 0;
    if (traffic_cache != queryVar.traffic_sign.recognition_result)
    {
        if (queryVar.traffic_sign.recognition_result == 2)
        {
            yellow_station = 0;
        }
        else if (queryVar.traffic_sign.recognition_result == 4)
        {
            yellow_station = 1;
        }

    }

    return yellow_station;
}

/*
 *  Decide Ego Vehicle located in which area
 *  @ param vehicle : CarInfo(), actual vehicle state
 *  @ param traffic : TrafficLightResult(), actual traffic state
 *  @ return area : int
 *  high velocity(v > 2m / s) :
 *      area = 0 : [v * (v / a + t_refresh), _] ->[soft braking line, _]
 *      area = 1 : [2, v * (v / a + t_refresh)] ->[crosswalk_pose_x_far, soft braking line]
 *      area = 2 : [-1, 2] ->[crosswalk_pose_x_near, crosswalk_pose_x_far]
 *      area = 3 : [_, -1] ->[_, crosswalk_pose_x_near]
 *  low velocity(v <= 2m / s) :
 *      area = 0 : [4, _] ->[slow_approach, _]
 *      area = 1 : [2, 4] ->[crosswalk_pose_x_far, slow_approach]
 *      area = 2 : [-1, 2] ->[crosswalk_pose_x_near, crosswalk_pose_x_far]
 *      area = 3 : [_, -1] ->[_, crosswalk_pose_x_near]
 */
int TrafficLightCondition::area_recognition() {
    // coordinate transformation
    double traffic_rel_pos = queryVar.RelativPositionX(queryVar.traffic_sign.pose.position) - radar_offset - carrosserie/2;

    int area = 1;
    double acc_brake_soft = acc_brake_soft_max;

    // compute soft braking deceleration
    if (queryVar.EgoVelocityX() <= 25.0 / 3.0)
    {
        // in case of slower than 30km / h
        acc_brake_soft = acc_brake_soft_max;
    }
    else if (queryVar.EgoVelocityX() >= 125.0 / 9.0)
    {
        // in case of faster than 50km / h
        acc_brake_soft = acc_brake_soft_min;
    }
    else
        // intermediate velocity phase[30km / h, 50km / h]
        acc_brake_soft = (acc_brake_soft_max + acc_brake_soft_min) / 2.0 - (acc_brake_soft_max - acc_brake_soft_min) / 2.0 * sin(PI / (125.0 / 9.0 - 25.0 / 3.0) * queryVar.egoCar.twist.linear.x);


    // default value(most cautious)
    if (queryVar.EgoVelocityX() > 2)
    {
        double soft_brake_boundary = pow(queryVar.EgoVelocityX() ,2) / acc_brake_soft
                + queryVar.EgoVelocityX()*t_refresh + s_emb_min;
        if (traffic_rel_pos > soft_brake_boundary)
            area = 0;
        else if (crosswalk_pose_x_far < traffic_rel_pos &&
                 traffic_rel_pos <= soft_brake_boundary)
            area = 1;
        else if (crosswalk_pose_x_near < traffic_rel_pos &&
                 traffic_rel_pos<= crosswalk_pose_x_far)
            area = 2;
        else
            area = 3;
    }
    else
    {
        if (traffic_rel_pos > slow_approach)
            area = 0;
        else if (crosswalk_pose_x_far < traffic_rel_pos &&
                 traffic_rel_pos <= slow_approach)
            area = 1;
        else if (crosswalk_pose_x_near < traffic_rel_pos &&
                 traffic_rel_pos <= crosswalk_pose_x_far)
            area = 2;
        else
            area = 3;
    }

    return area;
}

//from buffer

bool TrafficLightCondition::isBufferToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
    {
        transition = false;
    }
    else
    {
        if (area == 0 && motivation == 0)
            transition = true;
        else if (motivation == 0) // forwards drive
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isBufferToPrepLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 1)  // turn left
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isBufferToPrepRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 2)  // turn left
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isBufferToStop()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();

    if (collision)
    {
        transition = true;
    }

    return transition;
}

//from keep lane

bool TrafficLightCondition::isKeepLaneReflexive()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 0)
            transition = true;
        else if ((area == 1 || area == 2) &&
                 (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4 )))
            transition = true;
        else if (area == 3 && motivation == 0)
            transition = true;
        else
            transition = false;
    }
    return transition;
}


bool TrafficLightCondition::isKeepLaneToPrepLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 1)
            transition = true;
        else if ((area == 1 || area == 2) && motivation == 1 && (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4)))
            transition = true;
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isKeepLaneToPrepRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();


    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 2)
            transition = true;
        else if ((area == 1 || area == 2) && motivation == 2 && (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4)))
            transition = true;
        else if (area == 3 && motivation == 2)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isKeepLaneToStop()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();

    if (collision)
        transition = true;
    else
    {
        if ((area == 2 || area == 1) && ((queryVar.traffic_sign.recognition_result == 3) || (queryVar.traffic_sign.recognition_result == 2)))
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isKeepLaneToBuffer()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = true;
    else
    {
        if (motivation == 3 && area == 3)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//from stop

bool TrafficLightCondition::isStopReflexive()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();

    if (collision)
        transition = true;
    else
    {
        if ((area == 1 || area == 2) && ((queryVar.traffic_sign.recognition_result == 3) || (queryVar.traffic_sign.recognition_result == 2)))
            transition = true;
        else
            transition = false;
    }
    return transition;
}

//TODO
bool TrafficLightCondition::isStopToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (area == 0)
            transition = true;
        else if (area == 1 && (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4)))
            transition = true;
        else if (area == 2 && motivation == 0 && queryVar.traffic_sign.recognition_result == 1)
            // TODO not guarantee the yellow bulb
            transition = true;
        else if (area == 3 && motivation == 0)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isStopToPrepLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && motivation == 1 && queryVar.traffic_sign.recognition_result == 1)
            transition = true;
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isStopToPrepRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && motivation == 2 && queryVar.traffic_sign.recognition_result == 1)
            transition = true;
        else if (area == 3 && motivation == 2)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

//from prepare change left

bool TrafficLightCondition::isPrepLeftToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (motivation != 1)
            transition = true;
    }
    return transition;
}

bool TrafficLightCondition::isPrepLeftToStop()
{
    //initial
    bool transition = true;
    //condition same as isKeepLaneToStop(E5)
    transition = isKeepLaneToStop();

    return transition;
}

bool TrafficLightCondition::isPrepLeftToTurnLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && motivation == 1 && (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4)))
            transition = true;
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

//from prepare change right

bool TrafficLightCondition::isPrepRightToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (motivation != 2)
            transition = true;
    }
    return transition;
}

bool TrafficLightCondition::isPrepRightToStop()
{
    //initial
    bool transition = true;
    //condition same as isKeepLaneToStop(E5)
    transition = isKeepLaneToStop();

    return transition;
}

bool TrafficLightCondition::isPrepRightToTurnRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && motivation == 2 && (queryVar.traffic_sign.recognition_result == 1 || (queryVar.traffic_sign.recognition_result == 4)))
            transition = true;
        else if (area == 3 && motivation == 2)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

//from turn left

bool TrafficLightCondition::isTurnLeftToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (motivation == 3 && area == 3)
            transition = true;
        else
            transition = false;
    }
    return transition;


}

bool TrafficLightCondition::isTurnLeftToStop()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();


    if (collision)
        transition = true;
    else
        transition = false;

    return transition;

}

//from turn right

bool TrafficLightCondition::isTurnRightToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();

    if (collision)
        transition = false;
    else
    {
        if (motivation == 3 && area == 3)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool TrafficLightCondition::isTurnRightToStop()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();

    if (collision)
        transition = true;
    else
        transition = false;
    return transition;
}







