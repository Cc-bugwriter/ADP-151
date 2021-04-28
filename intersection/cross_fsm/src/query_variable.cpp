#include "cross_fsm/query_variable.h"


// adopting Vehicle Coordinate System, which is X: front, Y: left, Z: up
int queryVariable::getNearFront()const{
    auto n = objects.size();
    double dist_min = 999.9;
    double distance;
    int index = -1;

    double traffic_radius = sqrt(pow(RelativPositionX(traffic_sign.pose.position), 2) +
                                 pow(RelativPositionY(traffic_sign.pose.position), 2));
    double radius_threshold = EgoVelocityX()*2.0 > 7.0 ? EgoVelocityX()*2.0: 7.0;

    for(auto i = 0; i < n; i++){
        // coordinate transformation
        double entity_position_x_ = RelativPositionX(objects.at(i).pose.position);
        double entity_position_y_ = RelativPositionY(objects.at(i).pose.position);


        if (traffic_radius >= radius_threshold) {
            // far away by intersection, observe obstacles in straight direction
            if (entity_position_x_>0 && abs(entity_position_y_) < 1.75) {
                //now: in straight road try to get the nearst car in front of the ego without thinking about the direction
                distance =  entity_position_x_;
                if (distance < dist_min){
                    index = i;
                    dist_min = distance;
                }
            }
        } else {
            // near by intersection, observe obstacles in all direction
            if (entity_position_x_>0) {
                distance =  entity_position_x_;
                if (distance < dist_min){
                    index = i;
                    dist_min = distance;
                }
            }
        }
    }
    return index;
}

void queryVariable::proccessData(){
    int front = getNearFront();
    if(front == -1){
        // no objects nearby
        geometry_msgs::Point pose_inVeh = getDefaultPose_inVeh();
        geometry_msgs::Twist vel_inVeh = getDefaultVel_inVeh();
        dec_Object.pose.position.x = MapPositionX(pose_inVeh);
        dec_Object.pose.position.y = MapPositionY(pose_inVeh);
        dec_Object.pose.position.z = 0.0;
        dec_Object.velocity.linear.x = MapVelocityX(vel_inVeh);
        dec_Object.velocity.linear.y = MapVelocityY(vel_inVeh);
        dec_Object.velocity.linear.z = 0.0;
    }
    else{
        dec_Object = objects.at(front);
    }
}

int queryVariable::getSecurityCheckObstacle(cross_stopline stopline)const{
    auto n = objects.size();
    double dist_min = 999.9;
    double distance;
    int index = -1;
    std::cout<<"stopline position "<< stopline.x <<std::endl;


    for(auto i = 0; i < n; i++){
        double entity_position_x_ = objects.at(i).pose.position.x*(cos(stopline.angle)) + objects.at(i).pose.position.y*(sin(stopline.angle))
                - cos(stopline.angle)*stopline.x - sin(stopline.angle)*stopline.y;
        double entity_position_y_ = - objects.at(i).pose.position.x*sin(stopline.angle) + objects.at(i).pose.position.y*cos(stopline.angle)
                + sin(stopline.angle)*stopline.x - cos(stopline.angle)*stopline.y;
        std::cout<<"entity_position_x:   "<< entity_position_x_ <<std::endl;
//        std::cout<<"entity_position_y:   "<< entity_position_y_ <<std::endl;

        if (entity_position_x_> -30 && entity_position_x_< 70 &&abs(entity_position_y_) < 1.75) {
            //now: in straight road try to get the nearst car in front of the ego without thinking about the direction
            distance =  entity_position_x_;
            if (distance < dist_min){
                index = i;
                dist_min = distance;
            }
        }
    }
    return index;
}

void queryVariable::proccessDataSecuritycheck_left(){
    int securitycheckexist = getSecurityCheckObstacle(this->left_stopline);
//    std::cout<<"securitycheckexist LEFT: "<< securitycheckexist <<std::endl;

    if(securitycheckexist == -1){
        // no objects nearby
        geometry_msgs::Point pose_inVeh = getDefaultPose_inVeh();
        geometry_msgs::Twist vel_inVeh = getDefaultVel_inVeh();
        securitycheck_Object_left.pose.position.x = MapPositionX(pose_inVeh);
        securitycheck_Object_left.pose.position.y = MapPositionY(pose_inVeh);
        securitycheck_Object_left.pose.position.z = 0.0;
        securitycheck_Object_left.velocity.linear.x = MapVelocityX(vel_inVeh);
        securitycheck_Object_left.velocity.linear.y = MapVelocityY(vel_inVeh);
        securitycheck_Object_left.velocity.linear.z = 0.0;
    }
    else{
        securitycheck_Object_left = objects.at(securitycheckexist);
    }
}

void queryVariable::proccessDataSecuritycheck_right(){
    int securitycheckexist = getSecurityCheckObstacle(this->right_stopline);
    std::cout<<"securitycheckexist RIGHT: "<< securitycheckexist <<std::endl;

    if(securitycheckexist == -1){
        // no objects nearby
        geometry_msgs::Point pose_inVeh = getDefaultPose_inVeh();
        geometry_msgs::Twist vel_inVeh = getDefaultVel_inVeh();
        securitycheck_Object_right.pose.position.x = MapPositionX(pose_inVeh);
        securitycheck_Object_right.pose.position.y = MapPositionY(pose_inVeh);
        securitycheck_Object_right.pose.position.z = 0.0;
        securitycheck_Object_right.velocity.linear.x = MapVelocityX(vel_inVeh);
        securitycheck_Object_right.velocity.linear.y = MapVelocityY(vel_inVeh);
        securitycheck_Object_right.velocity.linear.z = 0.0;
    }
    else{
        securitycheck_Object_right = objects.at(securitycheckexist);
        std::cout<<"get securitycheck_Object_right "<< securitycheck_Object_right.pose.position.x <<std::endl;

    }
}

void queryVariable::proccessDataSecuritycheck_forward(){
    int securitycheckexist = getSecurityCheckObstacle(this->forward_stopline);
    std::cout<<"securitycheckexist FORWARD: "<< securitycheckexist <<std::endl;

    if(securitycheckexist == -1){
        // no objects nearby
        geometry_msgs::Point pose_inVeh = getDefaultPose_inVeh();
        geometry_msgs::Twist vel_inVeh = getDefaultVel_inVeh();
        securitycheck_Object_forward.pose.position.x = MapPositionX(pose_inVeh);
        securitycheck_Object_forward.pose.position.y = MapPositionY(pose_inVeh);
        securitycheck_Object_forward.pose.position.z = 0.0;
        securitycheck_Object_forward.velocity.linear.x = MapVelocityX(vel_inVeh);
        securitycheck_Object_forward.velocity.linear.y = MapVelocityY(vel_inVeh);
        securitycheck_Object_forward.velocity.linear.z = 0.0;
    }
    else{
        securitycheck_Object_forward = objects.at(securitycheckexist);
        std::cout<<"get securitycheck_Object_forward "<< securitycheck_Object_forward.pose.position.x <<std::endl;
    }
}


void queryVariable::callback1(const vaafo_msgs::CarInfo msg) {

    egoCar = msg;
}

void queryVariable::callback2(const vaafo_msgs::DetectedObjectArray msg) {

    objects = msg.objects;
}

void queryVariable::callback3(const vaafo_msgs::TrafficLightResult msg) {

    traffic_sign = msg;
    stop_sign.pose.position.x = -129.90;//map coordinate
    stop_sign.pose.position.y = -36.56;//map coordinate
    left_stopline.x = -999;
    left_stopline.y = -999;
    left_stopline.angle = -999;
    right_stopline.x = -154.96;
    right_stopline.y = -31.32;
    right_stopline.angle = 2.3152;//132.6569 degree
    forward_stopline.x = -158.57;
    forward_stopline.y = -53.07;
    forward_stopline.angle = 3.4552;//197.797 degree

}

void queryVariable::callback4(const vaafo_msgs::Lane msg) {
    ////TODO
}


vaafo_msgs::TrafficLightResult queryVariable::getTrafficLight()const {
    /*
    *  traffic light: recognition_result
    *   recognition_result = 1  -> green
    *   recognition_result = 2  -> green to yellow
    *   recognition_result = 3  -> red
    *   recognition_result = 4  -> red to yellow
    */

    return traffic_sign;
}


vaafo_msgs::CarInfo queryVariable::getEgocar()const {

    return egoCar;
}

vaafo_msgs::DetectedObject queryVariable::getDecObject()const {

    return dec_Object;
}

/*
 * nenerat a dummy target position
 * expression in vehicle coordinate
 * dummy target: replace the empty array of detected object
 *              avoid nan problem in following process
 */
geometry_msgs::Point queryVariable::getDefaultPose_inVeh()const {
    geometry_msgs::Point pose_inVeh;
    pose_inVeh.x = 999.9;
    pose_inVeh.y = 0.0;

    return pose_inVeh;
}

/*
 * nenerat a dummy target relative volecity
 * expression in vehicle coordinate
 * dummy target: replace the empty array of detected object
 *              avoid nan problem in following process
 */
geometry_msgs::Twist queryVariable::getDefaultVel_inVeh()const {
    geometry_msgs::Twist vel_inVeh;
    vel_inVeh.linear.x = 125.0 / 9.0 - EgoVelocityX();
    vel_inVeh.linear.y = 0.0 - EgoVelocityY();

    return vel_inVeh;
}

/*
 * get yaw angle of ego car
 * expression in map coordinate
 * yaw angle: angle between map coordinate x-axis and vehicle coordinate x-axis
 */
double queryVariable::getYaw()const {
    tf::Quaternion quat;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    return yaw;
}

/*
 * relative position from interested point to ego car origin
 * expression in vehicle coordinate x direction
 *  @ param pose_inMap : interested point position in map coordinate expression
 *  @ return position_x
 */
double queryVariable::RelativPositionX(geometry_msgs::Point pose_inMap)const {
    double yaw = getYaw();
    double position_x = pose_inMap.x*(cos(yaw)) + pose_inMap.y*(sin(yaw))
            - cos(yaw)*egoCar.pose.position.x - sin(yaw)*egoCar.pose.position.y;

    return position_x;
}

/*
 * relative position from interested point to ego car origin
 * expression in vehicle coordinate y direction
 *  @ param pose_inMap : interested point position in map coordinate expression
 *  @ return position_y
 */
double queryVariable::RelativPositionY(geometry_msgs::Point pose_inMap)const {
    double yaw = getYaw();
    double position_y = - pose_inMap.x*sin(yaw) + pose_inMap.y*cos(yaw)
            + sin(yaw)*egoCar.pose.position.x - cos(yaw)*egoCar.pose.position.y;

    return position_y;
}

/*
 * relative velocity from interested point to ego car origin
 * expression in vehicle coordinate x direction
 *  @ param vol_inMap : interested point velocity in map coordinate expression
 *  @ return velocity_x
 */
double queryVariable::RelativVelocityX(geometry_msgs::Twist vol_inMap)const {
    double yaw = getYaw();
    double velocity_x = vol_inMap.linear.x*cos(yaw) + vol_inMap.linear.y*sin(yaw)
            - cos(yaw)*egoCar.twist.linear.x - sin(yaw)*egoCar.twist.linear.y;

    return velocity_x;
}

/*
 * relative velocity from interested point to ego car origin
 * expression in vehicle coordinate y direction
 *  @ param vol_inMap : interested point velocity in map coordinate expression
 *  @ return velocity_y
 */
double queryVariable::RelativVelocityY(geometry_msgs::Twist vol_inMap)const {
    double yaw = getYaw();
    double velocity_y = - vol_inMap.linear.x*sin(yaw) + vol_inMap.linear.y*cos(yaw)
            + sin(yaw)*egoCar.twist.linear.x - cos(yaw)*egoCar.twist.linear.y;

    return velocity_y;
}

/*
 * velocity of ego car
 * expression in vehicle coordinate x direction
 */
double queryVariable::EgoVelocityX()const {
    double yaw = getYaw();
    double velocity_x = cos(yaw)*egoCar.twist.linear.x + sin(yaw)*egoCar.twist.linear.y;

    return velocity_x;
}

/*
 * velocity of ego car
 * expression in vehicle coordinate y direction
 */
double queryVariable::EgoVelocityY()const {
    double yaw = getYaw();
    double velocity_y = -sin(yaw)*egoCar.twist.linear.x + cos(yaw)*egoCar.twist.linear.y;

    return velocity_y;
}

/*
 * velocity of ego car
 * expression in vehicle coordinate x direction
 */
double queryVariable::EgoAccelerationX()const {
    double yaw = getYaw();
    double velocity_x = cos(yaw)*egoCar.acceleration.linear.x + sin(yaw)*egoCar.acceleration.linear.y;

    return velocity_x;
}

/*
 * velocity of ego car
 * expression in vehicle coordinate y direction
 */
double queryVariable::EgoAccelerationY()const {
    double yaw = getYaw();
    double velocity_y = -sin(yaw)*egoCar.acceleration.linear.x + cos(yaw)*egoCar.acceleration.linear.y;

    return velocity_y;
}

/*
 * position of interested point to map origin
 * expression in map coordinate x direction
 *  @ param pose_inVeh : interested point relative distance in vehicle coordinate expression
 *  @ return position_x
 */
double queryVariable::MapPositionX(geometry_msgs::Point pose_inVeh)const {
    double yaw = getYaw();
    double position_x = pose_inVeh.x*cos(yaw) - pose_inVeh.y*sin(yaw) + egoCar.pose.position.x;

    return position_x;
}

/*
 * position of interested point to map origin
 * expression in map coordinate y direction
 *  @ param pose_inVeh : interested point relative distance in vehicle coordinate expression
 *  @ return position_y
 */
double queryVariable::MapPositionY(geometry_msgs::Point pose_inVeh)const {
    double yaw = getYaw();
    double position_y = pose_inVeh.x*sin(yaw) + pose_inVeh.y*cos(yaw) + egoCar.pose.position.y;

    return position_y;
}

/*
 * velocity of interested point
 * expression in map coordinate x direction
 *  @ param vel_inVeh : interested point relative velocity in vehicle coordinate expression
 *  @ return velocity_x
 */
double queryVariable::MapVelocityX(geometry_msgs::Twist vel_inVeh)const {
    double yaw = getYaw();
    double velocity_x = vel_inVeh.linear.x*cos(yaw) - vel_inVeh.linear.y*sin(yaw) + egoCar.twist.linear.x;

    return velocity_x;
}

/*
 * velocity of interested point
 * expression in map coordinate y direction
 *  @ param vel_inVeh : interested point relative velocity in vehicle coordinate expression
 *  @ return velocity_y
 */
double queryVariable::MapVelocityY(geometry_msgs::Twist vel_inVeh)const {
    double yaw = getYaw();
    double velocity_y = vel_inVeh.linear.x*sin(yaw) + vel_inVeh.linear.y*cos(yaw) + egoCar.twist.linear.y;

    return velocity_y;
}

