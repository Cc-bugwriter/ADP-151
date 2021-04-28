#include "cross_fsm/stopsign_condition.h"
#include <cmath>

using namespace std;

StopSignCondition::StopSignCondition(queryVariable& qV): Condition(qV){
    qV.traffic_sign.pose.position.x = -129.90;//map coordinate
    qV.traffic_sign.pose.position.y = -36.56;//map coordinate

}

// static observe area(in meter)
double StopSignCondition::boundary_x_near = 6;
double StopSignCondition::boundary_x_far = 12;
double StopSignCondition::boundary_y_near_static = 1.5;
double StopSignCondition::boundary_y_far_static = 7.5;
double StopSignCondition::left_boundary_x_static = 0;
double StopSignCondition::right_boundary_x_static = -20;
double StopSignCondition::forward_boundary_x_static = -20;

// static observe area(in second)
double StopSignCondition::boundary_y_near_dynamic = 1.5;
double StopSignCondition::boundary_y_far_dynamic = 2.5;

int StopSignCondition::stop_catch = 0;



int StopSignCondition::motivation_detection() {
    ////TODO
    return 0;
}

// try exception
bool StopSignCondition::collision_check(){
    // check whether a collision appears in front of EgoCar

    // coordinate transformation
    double entity_position_x = queryVar.RelativPositionX(queryVar.dec_Object.pose.position) - radar_offset;
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

//    std::cout<<"getNearFront:dec_Object.pose.position.x: "<<queryVar.dec_Object.pose.position.x<<std::endl;
//    std::cout<<"getNearFront:dec_Object.pose.position.y: "<<queryVar.dec_Object.pose.position.y<<std::endl;

//    std::cout<<"getNearFront:entity_velocity_x: "<< entity_velocity_x<<std::endl;
//    std::cout<<"getNearFront:entity_velocity_y: "<< entity_velocity_y<<std::endl;
//    std::cout<<"getNearFront:entity_position_x: "<< entity_position_x<<std::endl;
//    std::cout<<"getNearFront:entity_position_y: "<< entity_position_y<<std::endl;

//    std::cout<<"collision_boundary_x: "<<collision_boundary_x<<std::endl;
//    std::cout<<"collision_boundary_y: "<<collision_boundary_y<<std::endl;

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

bool StopSignCondition::queue_check()
{
    //Check whether the ego vehicle in first queue of stop zone
    //	: param vehicle : CarInfo(), actual vehicle state
    //	: param entity : DetectedObject(), actual detected obstacle
    //	: param traffic : TrafficLightResult(), actual traffic state
    //	: return : bool, whether the ego vehicle in first position of stop queue

    double traffic_rel_pos = 0;
    traffic_rel_pos = queryVar.stop_sign.pose.position.x - queryVar.egoCar.pose.position.x- 5.0;

    if (traffic_rel_pos <= 1.5 * crosswalk_pose_x_far )
       {     std::cout<<"now first car at stop sign.......... "<<std::endl;
        return true;}
    else{

        return false;}
}

int StopSignCondition::stop_times_counter()
{
    //Count the actual stop times(v_ego == 0) in first stop queue position
    //	: param vehicle : CarInfo(), actual vehicle state
    //	: param entity : DetectedObject(), actual detected obstacle
    //	: param traffic : TrafficLightResult(), actual traffic state
    //	: return int : stop times

    bool first_queue = queue_check();
    std::cout<<"stop_times_counter...... "<<std::endl;

    if (first_queue &&
            abs(queryVar.egoCar.twist.linear.x) < 0.1)
    { 
        stop_catch = stop_catch + 1;
        std::cout<<"now counting stop time................ "<< stop_catch <<std::endl;
        return stop_catch;
    }
    else
        return 0;

}

bool StopSignCondition::check_right(vaafo_msgs::DetectedObject object){
    bool area_security = false;

    double entity_position_x_ = object.pose.position.x*(cos(queryVar.right_stopline.angle)) + object.pose.position.y*(sin(queryVar.right_stopline.angle))
            - cos(queryVar.right_stopline.angle)*queryVar.right_stopline.x - sin(queryVar.right_stopline.angle)*queryVar.right_stopline.y;

    double object_velocity_in_stopline = object.velocity.linear.x*cos(queryVar.right_stopline.angle) - object.velocity.linear.y*sin(queryVar.right_stopline.angle);
//    std::cout<<"check_right::boundary_y_dynamic * object_velocity: "<< boundary_y_near_dynamic * object_velocity_in_stopline <<std::endl;
//    std::cout<<"check_right::object_velocity: "<< object_velocity_in_stopline <<std::endl;
//    std::cout<< boundary_y_near_dynamic * abs(object_velocity_in_stopline) <<" >= "<< entity_position_x_ <<std::endl;

    if (boundary_y_near_dynamic * abs(object_velocity_in_stopline) >= entity_position_x_ && entity_position_x_ >= right_boundary_x_static){
        area_security = true;
        std::cout<<"area_security = true!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< area_security <<std::endl;
    }
    else
        area_security = false;


    return area_security;
}

bool StopSignCondition::check_left(vaafo_msgs::DetectedObject object){
    bool area_security = false;

    double object_velocity_in_stopline = object.velocity.linear.x*cos(queryVar.forward_stopline.angle) - object.velocity.linear.y*sin(queryVar.forward_stopline.angle);
    double entity_position_x_ = object.pose.position.x*(cos(queryVar.forward_stopline.angle)) + object.pose.position.y*(sin(queryVar.forward_stopline.angle))
            - cos(queryVar.forward_stopline.angle)*queryVar.right_stopline.x - sin(queryVar.forward_stopline.angle)*queryVar.right_stopline.y;

//    std::cout<<"check_forward::boundary_y_dynamic * object_velocity: "<< boundary_y_near_dynamic * object_velocity_in_stopline <<std::endl;
//    std::cout<<"check_forward::object_velocity: "<< object_velocity_in_stopline <<std::endl;
//    std::cout<< boundary_y_near_dynamic * abs(object_velocity_in_stopline) <<" >= "<<  entity_position_x_ <<std::endl;

    if (boundary_y_near_dynamic * abs(object_velocity_in_stopline) >= entity_position_x_ && entity_position_x_ >= forward_boundary_x_static){
        area_security = true;
        std::cout<<"area_security = true!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< area_security <<std::endl;
    }
    else
        area_security = false;

    return area_security;
}

bool StopSignCondition::check_forward(vaafo_msgs::DetectedObject object){
    bool area_security = false;

    double object_velocity_in_stopline = object.velocity.linear.x*cos(queryVar.forward_stopline.angle) - object.velocity.linear.y*sin(queryVar.forward_stopline.angle);
    double entity_position_x_ = object.pose.position.x*(cos(queryVar.forward_stopline.angle)) + object.pose.position.y*(sin(queryVar.forward_stopline.angle))
            - cos(queryVar.forward_stopline.angle)*queryVar.right_stopline.x - sin(queryVar.forward_stopline.angle)*queryVar.right_stopline.y;

//    std::cout<<"check_forward::boundary_y_dynamic * object_velocity: "<< boundary_y_near_dynamic * object_velocity_in_stopline <<std::endl;
//    std::cout<<"check_forward::object_velocity: "<< object_velocity_in_stopline <<std::endl;
//    std::cout<< boundary_y_near_dynamic * abs(object_velocity_in_stopline) <<" >= "<<  entity_position_x_ <<std::endl;

    if (boundary_y_near_dynamic * abs(object_velocity_in_stopline) >= entity_position_x_ && entity_position_x_ >= forward_boundary_x_static){
        area_security = true;
        std::cout<<"area_security = true!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    }
    else
        area_security = false;



    return area_security;
}

StopSignCondition::Security StopSignCondition::security_check()
{
    /*
    Check whether a collision appears in front of EgoCar
    : param vehicle : CarInfo(), actual vehicle state
    : param entity : DetectedObject(), actual detected obstacle
    : param traffic : TrafficLightResult(), actual traffic state
    : return security.forwards : bool, whether a collision appears in front area
    security.left : bool, whether a collision appears in left area
    security.right : bool, whether a collision appears in right area
    */


    security.right = check_right(queryVar.securitycheck_Object_right);
    std::cout<<"*************************security.right******************************"<< security.right <<std::endl;
//    std::cout<<"securitycheck right: "<< queryVar.securitycheck_Object_right.pose.position.x <<std::endl;
//    std::cout<<"securitycheck right: "<< queryVar.securitycheck_Object_right.y <<std::endl;
    security.left = check_left(queryVar.securitycheck_Object_left);
//    std::cout<<"securitycheck left: "<< queryVar.securitycheck_Object_left.x <<std::endl;
//    std::cout<<"securitycheck left: "<< queryVar.securitycheck_Object_right.y <<std::endl;
    security.forwards = check_forward(queryVar.securitycheck_Object_forward);
//    std::cout<<"securitycheck forward: "<< queryVar.securitycheck_Object_forward.pose.position.x <<std::endl;
//    std::cout<<"securitycheck forward: "<< queryVar.securitycheck_Object_right.y <<std::endl;

    return security;
}

int StopSignCondition::area_recognition()
{
    //Decide Ego Vehicle located in which area
    //	: param vehicle : CarInfo(), actual vehicle state
    //	: param traffic : TrafficLightResult(), actual traffic state
    //	: return area : int
    //	high velocity(v > 2m / s) :
    //	area = 0 : [v * (v / a + t_refresh), _] ->[soft braking line, _]
    //	area = 1 : [2, v * (v / a + t_refresh)] ->[crosswalk_pose_x_far, soft braking line]
    //	area = 2 : [-1, 2] ->[crosswalk_pose_x_near, crosswalk_pose_x_far]
    //	area = 3 : [_, -1] ->[_, crosswalk_pose_x_near]
    //	low velocity(v <= 2m / s) :
    //	area = 0 : [4, _] ->[slow_approach, _]
    //	area = 1 : [2, 4] ->[crosswalk_pose_x_far, slow_approach]
    //	area = 2 : [-1, 2] ->[crosswalk_pose_x_near, crosswalk_pose_x_far]
    //	area = 3 : [_, -1] ->[_, crosswalk_pose_x_near]
//    queryVar.traffic_sign.pose.position.x = -129.90;//map coordinate
//    queryVar.traffic_sign.pose.position.y = -36.56;//map coordinate
//    std::cout<<"Stopsign relative position "<< queryVar.traffic_sign.pose.position.x <<std::endl;
//    std::cout<<"Stopsign relative position "<< queryVar.traffic_sign.pose.position.y <<std::endl;

    double traffic_rel_pos = queryVar.RelativPositionX(queryVar.stop_sign.pose.position) - 5.0;
    int area = 1;
    double acc_brake_soft = acc_brake_soft_max;
    //std::cout<<"Stopsign relative position "<< traffic_rel_pos <<std::endl;

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
                 traffic_rel_pos <= crosswalk_pose_x_far)
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
        std::cout<<"area: "<<area<<std::endl;
    return area;
}

//from buffer

bool StopSignCondition::isBufferToKeepLane()
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

bool StopSignCondition::isBufferToPrepLeft()
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

bool StopSignCondition::isBufferToPrepRight()
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

bool StopSignCondition::isBufferToStop()
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

//?????
bool StopSignCondition::isKeepLaneReflexive()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();
    //??
    //security.forwards, security.left, security.right = \
    //	security_check()
    //	_ = stop_times_counter()


    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 0)
            transition = true;
        else if (area == 1 && stop_times > 0 && motivation == 0 )
        {
            if ((security.left == 0) || (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 2 && stop_times > 0 && motivation == 0)
        {
            if ((security.left == 0) || (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 3 && motivation == 0)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//??
bool StopSignCondition::isKeepLaneToPrepLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();
    //
    //_ = stop_times_counter()


    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 1)
            transition = false;
        else if (area == 1 && stop_times > 0 && motivation == 1)
            transition = true;
        else if (area == 2 && stop_times > 0 && motivation == 1)
        {
            if ((security.left == 0) || (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//??
bool StopSignCondition::isKeepLaneToPrepRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();
    //
    //_ = stop_times_counter()


    if (collision)
        transition = false;
    else
    {
        if (area == 0 && motivation == 2)
            transition = false;
        else if (area == 1 && stop_times > 0 && motivation == 2)
            transition = true;
        else if (area == 2 && stop_times > 0 && motivation == 2)
        {
            if ((security.left == 0) || (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 3 && motivation == 2)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//??
bool StopSignCondition::isKeepLaneToStop()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    bool first_queue = queue_check();
    security_check();
    int stop_times = stop_times_counter();

    //_ = stop_times_counter()
    //security.forwards, security.left, security.right = \
    //security_check()
    //_ = stop_times_counter()


    if (collision)
        transition = true;
    else
    {
        if (area == 1 && stop_times == 0)
        transition = true;
        else if (area == 2)
        {
            if (stop_times == 0 || (security.left || security.right))
                transition = true;
            else
                transition = false;
        }
        else
            transition = false;

    }

    return transition;
}

bool StopSignCondition::isKeepLaneToBuffer()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();

    if (collision)
    {
        if (motivation == 0)
        {
            transition = true;
            stop_catch = 0;
        }
        else
            transition = false;
    }

    return transition;
}

//from stop

//??
bool StopSignCondition::isStopReflexive()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();
    bool first_queue = queue_check();
    security_check();
    int stop_times = stop_times_counter();
    //security.forwards, security.left, security.right = \
    //	security_check()
    //_ = stop_times_counter()

    if (collision)
        transition = true;
    else
    {
        if ((area == 1 && first_queue) || area == 2)
        {
            if (motivation == 0 && (security.forwards || security.left || security.right))
                transition = true;
            else if (motivation == 1 && (security.forwards || security.left || security.right))
                transition = true;
            else if (motivation == 2 && (security.forwards || security.left))
                transition = true;
            else if (stop_times == 0)
                transition = true;
            else
                transition = false;
        }
        else
            transition = false;
    }

    return transition;
}

//TODO
bool StopSignCondition::isStopToKeepLane()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    bool motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();

    if (collision)
        transition = false;
    else
    {
        if (area == 0)
            transition = true;
        else if (area == 1 && stop_catch > 50 && ((security.left == 0) && (security.right == 0)))
            transition = true;
        else if (area == 2 && stop_catch > 50 && motivation == 0 && (security.right == 0)) //(area == 2 && stop_catch > 50 && motivation == 0 &&((security.left == 0) && (security.right == 0)))
            transition = true;
        else if (area == 3 && stop_catch > 50 && motivation == 0  && (security.right == 0)) //&& ((security.left == 0) && (security.right == 0))
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool StopSignCondition::isStopToPrepLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    bool motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();
    //security.forwards, security.left, security.right = \
    //	security_check()
    //_ = stop_times_counter()

    if (collision)
        transition = false;
    else
    {
        if (area == 3 && stop_times > 0 && motivation == 1 && ((security.left == 0) && (security.right == 0)))
            // TODO collision forwards should be improved
            transition = true;
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool StopSignCondition::isStopToPrepRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int motivation = motivation_detection();
    int area = area_recognition();
    security_check();
    int stop_times = stop_times_counter();
    //security.forwards, security.left, security.right = \
    //	security_check()
    //_ = stop_times_counter()

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && stop_times > 0 && motivation == 2 && (security.left == 0))
            transition = true;
        else if (area == 3 && motivation == 2 )
            transition = true;
        else
            transition = false;
    }
    return transition;
}

//from prepare change left

bool StopSignCondition::isPrepLeftToKeepLane()
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

bool StopSignCondition::isPrepLeftToStop()
{
    //initial
    bool transition = true;
    //condition same as isKeepLaneToStop(E5)
    transition = isKeepLaneToStop();

    return transition;
}

bool StopSignCondition::isPrepLeftToTurnLeft()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    bool motivation = motivation_detection();
    int area = area_recognition();
    bool first_queue = queue_check();
    security_check();
    int stop_times = stop_times_counter();
    //security.forwards, security.left, security.right = \
    //	security_check()
    //_ = stop_times_counter()

    if (collision)
        transition = false;
    else
    {
        if (area == 2 && first_queue && stop_times > 0 && motivation == 1)
        {
            if ((security.left == 0) && (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 2 && stop_catch > 0 && motivation == 1)
        {
            if ((security.left == 0) && (security.right == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 3 && motivation == 1)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//from prepare change right

bool StopSignCondition::isPrepRightToKeepLane()
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

bool StopSignCondition::isPrepRightToStop()
{
    //initial
    bool transition = true;
    //condition same as isKeepLaneToStop(E5)
    transition = isKeepLaneToStop();

    return transition;
}

bool StopSignCondition::isPrepRightToTurnRight()
{
    //initial
    bool transition = false;
    //compute condition variables
    bool collision = collision_check();
    int area = area_recognition();
    int motivation = motivation_detection();
    bool first_queue = queue_check();
    security_check();
    int stop_times = stop_times_counter();
    //security.forwards, security.left, security.right = \
    //	security_check()
    //_ = stop_times_counter()

    if (collision)
        transition = false;
    else
    {
        if (area == 1 && first_queue && stop_times > 0 && motivation == 2)
        {
            if ((security.left == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 2 && stop_times > 0 && motivation == 2)
        {
            if ((security.left == 0))
                transition = true;
            else
                transition = false;
        }
        else if (area == 3 && motivation == 2)
            transition = true;
        else
            transition = false;
    }

    return transition;
}

//from turn left

bool StopSignCondition::isTurnLeftToKeepLane()
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
        if (motivation == 1)
            transition = true;
        else
            transition = false;
    }
    return transition;


}

bool StopSignCondition::isTurnLeftToStop()
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

bool StopSignCondition::isTurnRightToKeepLane()
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
        if (motivation == 2)
            transition = true;
        else
            transition = false;
    }
    return transition;
}

bool StopSignCondition::isTurnRightToStop()
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
