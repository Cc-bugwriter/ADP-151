#include <ros/ros.h>
#include <vector>
#include <vaafo_msgs/Lane.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;



//#include "vaafo_msgs/waypoints.h"


void callback(const vaafo_msgs::Lane::ConstPtr &msgs)
{

    cout<<msgs->waypoints[0].pose.pose.position.x<<endl; 

}

int main(int argc, char ** argv)
{   
    ros::init(argc, argv,"sub");

    ros::NodeHandle nh1;

    ROS_INFO("111");

    ros::Subscriber sub=nh1.subscribe("path_info",1,callback);

     ROS_INFO("222");

    ros::spin();


    return 0;


}