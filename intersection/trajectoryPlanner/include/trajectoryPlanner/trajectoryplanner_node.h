#ifndef TRAJECTORYPLANNER_NODE_H
#define TRAJECTORYPLANNER_NODE_H

#include <ros/ros.h>
#include <vaafo_msgs/CarInfo.h>
#include <vaafo_msgs/DetectedObjectArray.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include "trajectoryPlanner/spline.h"
#include "trajectoryPlanner/quartic_polynomial.h"
#include "trajectoryPlanner/quintic_polynomial.h"
#include <vector>
#include <Eigen/Dense>
#include "vaafo_msgs/State.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<vaafo_msgs::CarInfo,vaafo_msgs::State,vaafo_msgs::DetectedObjectArray> MsgSyncPolicy;

typedef struct frenet_path {
    //path information in frenet coordinate
    std::vector<double> t;
    std::vector<double> d;
    std::vector<double> d_d;
    std::vector<double> d_dd;
    std::vector<double> d_ddd;
    std::vector<double> s;
    std::vector<double> s_d;
    std::vector<double> s_dd;
    std::vector<double> s_ddd;
    //coefficents of the cost functions
    double cd = 0.0;
    double cv = 0.0;
    double cf = 0.0;
    //path information in  global coordinate
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> v;
    std::vector<double> acc;
    std::vector<double> ds;
    std::vector<double> c;
} Frenet_path;

typedef std::vector <Frenet_path> FPLIST;

typedef struct coeff {
    double MAX_SPEED = 55.0 / 3.6;
    double MAX_ACCELERATION = 12.0;
    double MAX_CURVATURE = 1.0;
    double MAX_ROAD_WIDTH = 3.5;
    double D_ROAD_W = 1.0;
    double DT = 0.1;
    double MAX_T = 3.5;
    double MIN_T = 1.2;
    double D_T_S = 5.0 / 3.6;
    double N_S_SAMPLE = 1;
    double SAFETY_RADIUS = 2.0;
    double KJ = 0.1;
    double KT = 0.1;
    double KD = 1.0;
    double KLAT = 1.0;
    double KLON = 1.0;
} Coe;


class trajectoryPlanner_node {
public:
    trajectoryPlanner_node() = default;
    trajectoryPlanner_node(ros::NodeHandle &nh, const std::vector<double> &vx, const std::vector<double> &vy);

    virtual ~trajectoryPlanner_node();

    void frenetToGlobal(FPLIST &fplist);

    bool is_obstacle();
    bool check_collision(Frenet_path fp);

    FPLIST check_paths(FPLIST fplist);

    void optimalPath(ros::Publisher&);

    void FSMStateCallback(const vaafo_msgs::State::ConstPtr &msg);

    void cartToFrenet(const geometry_msgs::Pose &, const geometry_msgs::Twist &, double &, double &,double &,double &);
    void cartToFrenet(const geometry_msgs::Pose &, const geometry_msgs::Twist &, double &, double &);

    void s_targetCallback(const std_msgs::Float64 &msg);
    void d_targetCallback(const std_msgs::Float64 &msg);

    std::vector<double> arange(double start, double end, double interval) {
        std::vector<double> di{start};
        while (di.back() < end) {
            di.push_back(di.back() + interval);
        }
        di.pop_back();
        return di;
    }

    std::vector<double> di, Ti;
    Coe coefficent;
    std_msgs::Header header;
    void pub_msg(const Frenet_path&);
private:
    ros::NodeHandle nh_;
    ros::Subscriber nh_FSM_state;
    ros::Subscriber nh_deObj;
    ros::Subscriber nh_CarInfo;
    //TODO SUBSCRIBER
    ros::Publisher path;
    double acceleration;
    geometry_msgs::Pose cart_; //given cartesian points
    geometry_msgs::Twist twist_;
    geometry_msgs::Twist vcur;
    geometry_msgs::Pose posecur;
    geometry_msgs::Twist acc_cur;

    //double d_target;
    std::vector<double> way_x_, way_y_; /*rx_, ry_, ryaw_, rk_*/
    Spline csp_;
    std::vector <geometry_msgs::Pose> obstacles;
    double integral_error[5] = {0};
    size_t index_integral_error = 0;

    int FSM_state=2;
    double c_d;             //d_movement of the start point
    double c_d_d;           //d_velocity of the start point
    double c_d_dd=0.0;      //d_acceleration of the start point
    double target_d;
    double target_d_d;
    double s0;              // s_movement of the start point
    double c_speed;         //s_velocity of the start point
    double s_target=0.0;    //s_movement of the target point
    double s_d;             //s_velocity of the target point
    FPLIST calc_frenet_paths();
};

#endif // TRAJECTORYPLANNER_NODE_H
