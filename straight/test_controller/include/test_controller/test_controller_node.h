#ifndef TEST_CONTROLLER_NODE_H
#define TEST_CONTROLLER_NODE_H
#include <ros/ros.h>
#include <vaafo_msgs/CarInfo.h>

class pubCarInfo
{
public:
    pubCarInfo(ros::NodeHandle& nh);
    void pub();
    void generateData();

private:
    ros::Publisher carinfo_pub;
    vaafo_msgs::CarInfo c_msg;

};

#endif // TEST_CONTROLLER_NODE_H
