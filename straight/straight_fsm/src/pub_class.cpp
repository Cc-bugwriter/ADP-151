#include "ros/ros.h"
#include "straight_fsm/pub_class.h"
using namespace std;


pub::pub(ros::NodeHandle& nh) {
    chatter_pubstate = nh.advertise<vaafo_msgs::State>("behavior_state", 10);
}

void pub::pubState(const fsm_state* fs) {
    vaafo_msgs::State pub_state;
    pub_state.state_id = fs->turn2int();
    cout<<"the state id is:"<<pub_state.state_id<<endl;
    pub_state.header = fs->egoCar.header;
    pub_state.target.acceleration = goalacceleration();
    //pub_state.target.twist = goalvelocity(fs, qv);
    pub_state.target.twist = fs->egoCar_target.twist;
    pub_state.target.pose = fs->egoCar_target.pose;

    chatter_pubstate.publish(pub_state);
}



geometry_msgs::Twist pub::goalacceleration() {
    geometry_msgs::Twist goalAcc;
    goalAcc.linear.x = 0.0;
    goalAcc.linear.y = 0.0;
    goalAcc.linear.z = 0.0;
    goalAcc.angular.x = 0.0;
    goalAcc.angular.x = 0.0;
    goalAcc.angular.x = 0.0;

    return goalAcc;
}


