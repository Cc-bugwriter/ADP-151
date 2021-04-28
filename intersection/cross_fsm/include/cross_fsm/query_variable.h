#ifndef QUERY_VARIABLE_H
#define QUERY_VARIABLE_H
#include <ros/ros.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/DetectedObject.h>
#include <vaafo_msgs/DetectedObjectArray.h>
#include <vaafo_msgs/TrafficLightResult.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <tf/tf.h>


class queryVariable
{
    friend class RechtvorLinksCondition;
    friend class StopSignCondition;
    friend class TrafficLightCondition;


private:
    //ros::NodeHandle nh;
    ros::Subscriber sub_carinfo;
    ros::Subscriber sub_detectedobject;
    ros::Subscriber sub_trafficlightresult;
    ros::Subscriber sub_lane;



    //transformed data for ego car from vaafo_msgs::CarInfo
    int ego_lane_id;
    vaafo_msgs::CarInfo egoCar;

    //TODO:transformed data for object car from vaafo_msgs::DetectedObjectArray
    //TODO:just one detected object?
    std::vector<vaafo_msgs::DetectedObject> objects;
    vaafo_msgs::DetectedObject dec_Object;
    vaafo_msgs::DetectedObject securitycheck_Object_left;
    vaafo_msgs::DetectedObject securitycheck_Object_right;
    vaafo_msgs::DetectedObject securitycheck_Object_forward;

    //transformed data for trafficsign from vaafo_msgs::TrafficlightResult
    vaafo_msgs::TrafficLightResult traffic_sign;
    vaafo_msgs::TrafficLightResult stop_sign;

    //TODO:transformed data for lane from vaafo_msgs::Lane

    // empty detected
    geometry_msgs::Point getDefaultPose_inVeh()const;
    geometry_msgs::Twist getDefaultVel_inVeh()const;

    //crossroad stopline position and angle
    struct cross_stopline {
        double x;
        double y;
        double angle;
    };

    cross_stopline left_stopline;
    cross_stopline right_stopline;
    cross_stopline forward_stopline;


public:
    queryVariable(ros::NodeHandle& nh)
    {
    sub_carinfo= nh.subscribe("VechInfo", 10, &queryVariable::callback1, this);
    sub_detectedobject = nh.subscribe("finalObjList/map/objects", 10, &queryVariable::callback2, this);
    sub_trafficlightresult = nh.subscribe("traffic_light", 10, &queryVariable::callback3, this);
    sub_lane = nh.subscribe("Lane", 10, &queryVariable::callback4, this);


    }

    ~queryVariable(){
    }


    void callback1(const vaafo_msgs::CarInfo msg);
    void callback2(const vaafo_msgs::DetectedObjectArray msg);
    void callback3(const vaafo_msgs::TrafficLightResult msg);
    void callback4(const vaafo_msgs::Lane msg);
    void callback5(const vaafo_msgs::Lane msg);

    vaafo_msgs::TrafficLightResult getTrafficLight()const;
    vaafo_msgs::CarInfo getEgocar()const;
    vaafo_msgs::DetectedObject getDecObject()const;

    // get the index of the array from detected object array
    int getNearFront()const;
    void proccessData();
    int getSecurityCheckObstacle(cross_stopline)const;
    void proccessDataSecuritycheck_left();
    void proccessDataSecuritycheck_right();
    void proccessDataSecuritycheck_forward();

    // coordinate transformation
    double getYaw()const;
    double RelativPositionX(geometry_msgs::Point)const;
    double RelativPositionY(geometry_msgs::Point)const;
    double RelativVelocityX(geometry_msgs::Twist)const;
    double RelativVelocityY(geometry_msgs::Twist)const;

    double EgoVelocityX()const;
    double EgoVelocityY()const;
    double EgoAccelerationX()const;
    double EgoAccelerationY()const;

    double MapPositionX(geometry_msgs::Point)const;
    double MapPositionY(geometry_msgs::Point)const;
    double MapVelocityX(geometry_msgs::Twist)const;
    double MapVelocityY(geometry_msgs::Twist)const;





};
#endif // QUERY_VARIABLE_H
