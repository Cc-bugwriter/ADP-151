#include <test_controller/test_controller_node.h>

pubCarInfo::pubCarInfo(ros::NodeHandle& nh)
{
    carinfo_pub = nh.advertise<vaafo_msgs::CarInfo>("cmd_signal", 1);
}

void pubCarInfo::generateData(){
    c_msg.twist.linear.x = 100;
    c_msg.twist.linear.y = 0;
}

void pubCarInfo::pub(){
    ros::Rate loop_rate(10);
    while(ros::ok){
        generateData();
        carinfo_pub.publish(c_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argv, char** argc){
    ros::init(argv, argc, "cmd_signal");
    ros::NodeHandle nh;
    pubCarInfo pub233(nh);
    pub233.pub();
    return 0;

}
