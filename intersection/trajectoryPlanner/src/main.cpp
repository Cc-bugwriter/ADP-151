//#define EIGEN_USE_MKL_ALL
//#define EIGEN_VECTORIZE_SSE4_2
#include "trajectoryPlanner/cubicSpline.h"
#include "trajectoryPlanner/quartic_polynomial.h"
#include "trajectoryPlanner/quintic_polynomial.h"
#include "trajectoryPlanner/trajectoryplanner_node.h"
#include <ros/ros.h>
#include <vector>
#include <vaafo_msgs/Lane.h>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

using namespace std;
using namespace chrono;

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajTalker");
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::NodeHandle nh_;
    //reference line
    FPLIST fp;

    //read waypoints from txt file
//    std::ifstream infile("/home/cui/catkin_WS/cross_fsm/src/trajectoryPlanner/path_file/interval40cm_right.txt");
    std::ifstream infile("/home/cui/catkin_WS/cross_fsm/src/trajectoryPlanner/path_file/interval40cm_straight.txt");
    vector<double> x_cord;
    vector<double> y_cord;

    string line;

    while (getline(infile, line))
    {
        istringstream iss(line);
        double a, b;

        if (!(iss >> a >> b)) break;// error

        x_cord.push_back(a);
        y_cord.push_back(b);
    }

//    auto start=system_clock::now();
//    Spline csp(waypoint_x, waypoint_y);
//    auto end=system_clock::now();
//    auto duration = duration_cast<microseconds>(end - start);
//    cout << double(duration.count()) * microseconds::period::num / microseconds::period::den << endl;

    ros::Publisher pub = nh.advertise<vaafo_msgs::CarInfo>("/cmd_signal", 10);
    ros::Rate loop_rate(10);
    trajectoryPlanner_node trajectory(n, x_cord, y_cord);
    while (ros::ok()) {
        trajectory.optimalPath(pub);
        ROS_INFO("running");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
