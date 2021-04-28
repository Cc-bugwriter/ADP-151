#define PI 3.141592653589793238462643383279502884 /* pi */

#include <ros/ros.h>
#include "trajectoryPlanner/trajectoryplanner_node.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <tf/tf.h>

using namespace std;

trajectoryPlanner_node::trajectoryPlanner_node(ros::NodeHandle &nh, const std::vector<double> &way_x, const std::vector<double> &way_y)
    : nh_(nh), way_x_(way_x), way_y_(way_y) {

    //give the sampling data vector di and Ti for trajectory generation
    di = arange(-coefficent.MAX_ROAD_WIDTH, coefficent.MAX_ROAD_WIDTH, coefficent.D_ROAD_W);

    csp_=Spline(way_x_,way_y_);

    //subscribe msg from bahevior planner
    nh_FSM_state = nh_.subscribe("/behavior_state", 10, &trajectoryPlanner_node::FSMStateCallback, this);

    double meanVelocity=(c_speed+s_d)/2.0;
    double pathDistance=s_target-s0;
    if (meanVelocity < 0.1){
        Ti = arange(coefficent.MIN_T - 1.0, coefficent.MAX_T, coefficent.DT);
    }
    else{
        double time=pathDistance/meanVelocity;
        double minTime=time-0.5 > 0 ? time-0.5 : 0.2;
        double maxTime=time+0.5;
        Ti = arange(minTime, maxTime, coefficent.DT);
    }
}

trajectoryPlanner_node::~trajectoryPlanner_node(){}


//callback function for subscriber nh_CarInfo
void trajectoryPlanner_node::FSMStateCallback(const vaafo_msgs::State::ConstPtr &msg) {
    vcur=msg->ego_car.twist;
    posecur=msg->ego_car.pose;
    acc_cur=msg->ego_car.acceleration;
    header=msg->ego_car.header;
    cartToFrenet(posecur,vcur,c_d,c_d_d,s0,c_speed);
    twist_=msg->target.twist;
    acceleration = sqrt(pow(msg->target.acceleration.linear.x, 2) + pow(msg->target.acceleration.linear.y, 2));
    cart_=msg->target.pose;
    FSM_state=msg->state_id;
    cartToFrenet(cart_, twist_, s_target, s_d);
    s_target = msg->target.s_d.x;
    obstacles.push_back(msg->obstacle.pose);
}

void trajectoryPlanner_node::cartToFrenet(const geometry_msgs::Pose& cart, const geometry_msgs::Twist& twist, double &d, double &d_d, double &s, double &vs){
    std::vector<double> dis;
    double theta_r;
    for (size_t i = 0; i < way_x_.size(); i++) {
        //calculate the distance between cart point and reference line
        dis.push_back(sqrt(std::pow(way_x_[i]-cart.position.x,2))+std::pow(way_y_[i]-cart.position.y,2));
    }
    //find the index of the reference point (replace corresponding reference with the nearst point)
    size_t idx=std::min_element(dis.begin(),dis.end())-dis.begin(); //under <algorithm>
    theta_r=csp_.getYaw(csp_.S.at(idx));

    if ((cart.position.y-way_y_[idx])*cos(theta_r)-(cart.position.x-way_x_[idx])*sin(theta_r)>0){
        d=dis[idx]; //d movement
    }
    else {
        d=-dis[idx]; //d movement
    }

    s=csp_.S[idx];
    double theta_x=tf::getYaw(cart.orientation);
    double delta_theta=theta_x-theta_r;
    double vx=sqrt(std::pow(twist.linear.x,2)+std::pow(twist.linear.y,2));
    d_d=vx*sin(delta_theta); //dd/dt
    double curv=csp_.getCurvature(csp_.S.at(idx)); //kappa_r
    vs=vx*cos(delta_theta)/(1-curv*d); //ds/dt

}

void trajectoryPlanner_node::cartToFrenet(const geometry_msgs::Pose &p, const geometry_msgs::Twist &v, double &s_end, double & v_s_end){
    std::vector<double> dis;
    double theta_r;
    for (size_t i = 0; i < way_x_.size(); i++) {
        //calculate the distance between cart point and reference line
        dis.push_back(sqrt(std::pow(way_x_[i] - p.position.x,2)) + std::pow(way_y_[i] - p.position.y,2));
    }
    //find the index of the reference point (replace corresponding reference with the nearst point)
    size_t idx = std::min_element(dis.begin(), dis.end()) - dis.begin(); //under <algorithm>
    theta_r=csp_.getYaw(csp_.S.at(idx));
    s_end = csp_.S[idx];
    double theta_x=atan2(v.linear.y, v.linear.x);
    double delta_theta=theta_x - theta_r;
    double vx=sqrt(std::pow(v.linear.x, 2)+std::pow(v.linear.y, 2));
    v_s_end=vx*cos(delta_theta); //ds/dt

}

//calculate trajectories in frenet coordinate using the sampling data vector di and Ti
FPLIST trajectoryPlanner_node::calc_frenet_paths() {
//    ROS_INFO("calc_frenet_paths");
    FPLIST frenet_paths;

    //di loop for different laternal movement
    for (auto di_ : di) {
        // every di correspond to all Ti_ in Ti
        for (auto Ti_ : Ti) {
            Frenet_path fp;
            // using quintic polynomial to generate laternal movement with certain Ti_
            Quintic_polynomial lat_qp(c_d, c_d_d, c_d_dd, di_, 0.0, 0.0, Ti_);
            fp.t = arange(0.0, Ti_, coefficent.DT);
            //fp.d=listInit(&Quintic_polynomial::calcPoint,fp.t);
            for (auto t:fp.t) {
                fp.d.push_back(lat_qp.calcPoint(t));
                fp.d_d.push_back(lat_qp.calc1D(t));
                fp.d_dd.push_back(lat_qp.calc2D(t));
                fp.d_ddd.push_back(lat_qp.calc3D(t));
            }
            //according to the FSM_state given by Behavior Planner, calculate S in frenet coordinate using different polynomials
            
            if (FSM_state != 7) {
                double deltas = 0.1;
                //double tv = sqrt(pow(twist.linear.x,2)+pow(twist.linear.y,2));
                std::vector<double> s_end = arange(s_target - deltas, s_target + deltas, coefficent.D_T_S);
                for (auto s_target_:s_end) {
                    Frenet_path cfp=fp;
                    Quintic_polynomial lon_qp(s0, c_speed, acc_cur.linear.x, s_target_, s_d, 0.0, Ti_);
                    for (auto t:fp.t) {
                        cfp.s.push_back(lon_qp.calcPoint(t));
                        cfp.s_d.push_back(lon_qp.calc1D(t));
                        cfp.s_dd.push_back(lon_qp.calc2D(t));
                        cfp.s_ddd.push_back(lon_qp.calc3D(t));
                    }
                    double Jd = 0;
                    for (auto i:fp.d_ddd) {
                        Jd += std::pow(i,2);
                    }
                    double Js = 0;
                    for (auto i:fp.s_ddd) {
                        Js += std::pow(i,2);
                    }
                    double S = pow((s_target - cfp.s.back()), 2);
                    cfp.cd = coefficent.KJ * Jd + coefficent.KT * Ti_ + coefficent.KD * pow(cfp.d.back()-(-1.75), 2);
                    cfp.cv = coefficent.KJ * Js + coefficent.KT * Ti_ + coefficent.KD * S;
                    cfp.cf = coefficent.KLAT * cfp.cd + coefficent.KLON * cfp.cv;
                    frenet_paths.push_back(cfp);
                }
            }

            //FSM_state=7,stop the viehcle
            else if (FSM_state == 7) {
                double deltas = 0.1;
                std::vector<double> s_end = arange(s_target - deltas, s_target + deltas, coefficent.D_T_S);
                for (auto s_target_:s_end) {
                    Frenet_path cfp=fp;
                    Quintic_polynomial lon_qp(s0, c_speed, acc_cur.linear.x, s_target_, s_d, 0.0, Ti_);
                    for (auto t:fp.t) {
                        cfp.s.push_back(lon_qp.calcPoint(t));
                        cfp.s_d.push_back(lon_qp.calc1D(t));
                        cfp.s_dd.push_back(lon_qp.calc2D(t));
                        cfp.s_ddd.push_back(lon_qp.calc3D(t));
                    }
                    double Jd = 0;
                    for (auto i:fp.d_ddd) {
                        Jd += std::pow(i,2);
                    }
                    double Js = 0;
                    for (auto i:fp.s_ddd) {
                        Js += std::pow(i,2);
                    }
                    double ds = pow((s_d- cfp.s_d.back()), 2);
                    cfp.cd = coefficent.KJ * Jd + coefficent.KT * Ti_ + coefficent.KD * pow(cfp.d.back()-(-1.75), 2);
                    cfp.cv = coefficent.KJ * Js + coefficent.KT * Ti_ + coefficent.KD * ds;
                    cfp.cf = coefficent.KLAT * cfp.cd + coefficent.KLON * cfp.cv;
                    frenet_paths.push_back(cfp);
                }
            }
            //TODO
            //more FSM_state to be definited

        }
    }
    return frenet_paths;
}

/*
 *  tranfer the trajectories which are in frenet coordinate to glaobal coordinate
 *  now all the possible trajectories are stored in FPLIST
 */
void trajectoryPlanner_node::frenetToGlobal(FPLIST &fplist) {
    /*
 * if there is no trajectory in fplist then do nothing, otherwise transfer the trajectories from frenet coordinate to glaobal coordinate
*/
//    ROS_INFO("frenetToGlobal");
    if (fplist.empty()) {
        ROS_INFO("No trajectories generated");
    } else {
        for (auto &fp : fplist) {
            for (size_t i = 0; i < fp.s.size(); ++i) {
                double ix = csp_.getX(fp.s.at(i));
                double iy = csp_.getY(fp.s.at(i));
                if (!ix) {
                    break;
                }
                double iyaw = csp_.getYaw(fp.s.at(i));
                double di = fp.d.at(i);
                double fx = ix + di * cos(iyaw + PI / 2.0);
                double fy = iy + di * sin(iyaw + PI / 2.0);
                fp.x.push_back(fx);
                fp.y.push_back(fy);
            }
//            if(fp.x.size()<=3){
                // quintic approximation
                for (size_t i = 0; i < fp.x.size()-1; ++i) {
                    double dx = fp.x.at(i+1) - fp.x.at(i);
                    double dy = fp.y.at(i+1) - fp.y.at(i);
                    fp.yaw.push_back(atan2(dy, dx));
                    fp.ds.push_back(sqrt(pow(dx, 2) + pow(dy, 2)));
                }
                fp.yaw.push_back(fp.yaw.back());
                fp.ds.push_back(fp.ds.back());

                for (size_t i = 0; i < fp.yaw.size() - 1; ++i) {
                    double curvature = (fp.yaw.at(i+1) - fp.yaw.at(i)) / fp.ds.at(i);
                    fp.c.push_back(curvature);
                }
//            }
//            else{
//                // cubic interpolation
//                Frenet_path tfp = fp;
//                Spline temp(tfp.x,tfp.y);
//                for (size_t i = 0; i < fp.x.size(); ++i) {
//                    fp.yaw.push_back(temp.getYaw(temp.S.at(i)));
//                    fp.ds.push_back(temp.S.at(i));
//                    fp.c.push_back(temp.getCurvature(temp.S.at(i)));
//                }
//            }
            

            for (size_t i = 0; i < fp.yaw.size(); ++i) {
                double delta = fp.yaw.at(i)-csp_.getYaw(fp.s.at(i));
//                if(delta < 0.1){
                    double curv=csp_.getCurvature(fp.s.at(i));
                    double v=fp.s_d.at(i) * (1-curv*fp.d.at(i)) / cos(delta);
                    fp.v.push_back(v);
//                }
//                else{
//                    double v = fp.d_d.at(i) / sin(delta);
//                    fp.v.push_back(v);
//                }
            }

        }
    }
}

bool trajectoryPlanner_node::check_collision(Frenet_path fp) {
    //when no obstacles, no collision, return false
    if (obstacles.empty()) return false;
    else { //have obstacles
        std::vector<double> dis;
        //calculate all the distances of the obstacles
        for (geometry_msgs::Pose obstacle: obstacles) {
            for (size_t i = 0; i < fp.x.size(); ++i) {
                double temp = sqrt(pow(fp.x.at(i) - obstacle.position.x, 2) + pow(fp.y.at(i) - obstacle.position.y, 2));
                dis.push_back(temp);
            }
            //if exist one distance which is smaller than the safety_radius, return true
            bool collision = any_of(dis.begin(), dis.end(), [this](double i){ return i < coefficent.SAFETY_RADIUS; });
            //when collision is true, then return true, which means collison happened
            if (collision) return true;
        }
        //if no collision happened, return false
        return false;
    }
}

/*
 * trajectory should satisfy the basic rules:
 *     1) the max_speed of the trajectory cannot larger than the MAX_SPEED
 *     2) the max_acceleration of the trajectory cannot larger than the MAX_ACCELERATION
 *     3) the max_curvature of the trajectory canot larger than the MAX_CURVATURE
 *     if there is a speed of one trajectory larger than the allowance, then continue the loop
 */
FPLIST trajectoryPlanner_node::check_paths(FPLIST fplist) {
//    ROS_INFO("check_paths");
    FPLIST goodPath;
    std::vector<size_t> idx;

    for (size_t i = 0; i < fplist.size(); ++i) {
        if (std::any_of(fplist[i].v.begin(), fplist[i].v.end(), [this](double i){ return abs(i) > coefficent.MAX_SPEED; })) continue;
        //if the speed is satisfied, then check the acceleration, if not satisfied, then continue the loop
//        else if (std::any_of(fplist[i].s_dd.begin(), fplist[i].s_dd.end(), [this](double i){ return abs(i) > coefficent.MAX_ACCELERATION; })) continue;
        //if the speed and acceleration are satisfied, then check the curvature, if not satisfied, then continue the loop
//        else if (std::any_of(fplist[i].c.begin(), fplist[i].c.end(), [this](double i){ return abs(i) > coefficent.MAX_CURVATURE; })) continue;
        //if the kinematic constraints are satisfied, then check the collision, if collision, continue the loop
        //else if (check_collision(fplist[i])) continue;

        idx.push_back(i);
    }

    //push the satisfied frenet path in the goodpath vector
    for (auto i : idx) goodPath.push_back(fplist[i]);

    return goodPath;
}

void trajectoryPlanner_node::optimalPath(ros::Publisher& pub) {
//    ROS_INFO("optimalPath");
    FPLIST fplist = calc_frenet_paths();
    cout<<"insgesamte "<<fplist.size()<<" traj generated"<<endl;
    frenetToGlobal(fplist);
    FPLIST goodPath = check_paths(fplist);
    cout<<"there are "<<goodPath.size()<<" traj"<<endl;
    Frenet_path bestfp;
    double mincost = 1000000.0;
    for (auto i:goodPath) {
        if (mincost >= i.cf) {
            mincost = i.cf;
            bestfp = i;
        }
    }
    
    if(bestfp.x.empty()){
        ROS_INFO("No trajectories generated in Cartesian Coordinate");
    }
    else {
        ROS_INFO("Trajectories generated");
        vaafo_msgs::CarInfo msg;
        size_t index_yaw = bestfp.x.size() < 7 ? bestfp.x.size() - 1 : 5;       // -> yaw
        size_t index_vel = index_yaw;     // -> velocity
//        double yaw_soll = std::accumulate(bestfp.yaw.begin(), bestfp.yaw.begin() + index_yaw, 0.0) / double(index_yaw);
        double yaw_soll = bestfp.yaw.at(index_yaw);
        double yaw_ist = tf::getYaw(posecur.orientation);
        double yaw_P = yaw_soll - yaw_ist;
        yaw_P = yaw_P > PI ? yaw_P - 2*PI : yaw_P;
        msg.pose.position.x = bestfp.x[index_vel];
        msg.pose.position.y = bestfp.y[index_vel];
        cout << "target position s : " << s_target << endl;
        msg.twist.linear.x = 3.6 * bestfp.v[index_vel] * cos(yaw_soll);
        msg.twist.linear.y = 3.6 * bestfp.v[index_vel] * sin(yaw_soll);
//        cout << "target speed x= " << msg.twist.linear.x << endl;
//        cout << "target speed y= " << msg.twist.linear.y << endl;
        cout << "delta yaw_soll : " << yaw_soll * 180.0 / PI << endl;
        cout << "delta yaw_ist : " << yaw_ist * 180.0 / PI << endl;
        cout << "delta yaw angle : " << yaw_P * 180.0 / PI << endl;
        /*
         *  yaw_P description
         *  positive:   turn left
         *  negative:   turn right
         *  the yaw_P orientation has identical direction as map coordinate z axis
         */
        if ((abs(yaw_P) > 0.5  ) || isnan(yaw_P) || abs(c_d) > 0.3) {
            if (!isnan(yaw_P)) {
                if (index_integral_error != 5) {
                    integral_error[index_integral_error] = yaw_P;
                    index_integral_error ++;
                } else {
                    index_integral_error = 0;
                    integral_error[index_integral_error] = yaw_P;
                }
            }
            // lateral offset dominate
            cout << "lateral offset: " << c_d << endl;
            double vel_current = sqrt(pow(vcur.linear.x, 2) + pow(vcur.linear.y, 2));
            double omiga_current = vcur.angular.z;
            if (isnan(vel_current / omiga_current)) {
                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0), msg.pose.orientation);
            } else {
                cout << "omiga : " << omiga_current << endl;
                cout << "turning radius: " << abs(vel_current / omiga_current) << endl;
                cout << "controll angle: " << c_d/-abs(vel_current / omiga_current) * 180.0 / PI << endl;
                if (abs(omiga_current) < 0.3) {
                    // straight drive
                    if (abs(c_d) < 0.35) {
                        ROS_INFO("normal lateral offset control");
                        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(c_d/-20.0), msg.pose.orientation);
                    }
                    else {
                        ROS_INFO("super lateral offset control");
                        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(c_d/-4.0), msg.pose.orientation);
                    }
                }
                else {
                    // turning
                    ROS_INFO("curve rate control");
                    double curve_control = 12.0*c_d/-abs(vel_current / omiga_current);
                    curve_control = curve_control > PI ? (PI - 0.01)/1.7 : curve_control;
                    curve_control = curve_control < -PI ? (- PI + 0.01)/1.7 : curve_control;
                    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(curve_control), msg.pose.orientation);
                }
            }
        } else {
            // PI control -> yaw dominate
            ROS_INFO("PID");
            if (index_integral_error != 5) {
                integral_error[index_integral_error] = yaw_P;
                index_integral_error ++;
            } else {
                index_integral_error = 0;
                integral_error[index_integral_error] = yaw_P;
            }
            double yaw_I = std::accumulate(integral_error, integral_error + 4, 0.0) - yaw_P; // -> not involve current P error
            cout << "integral yaw angle : " << yaw_I * 180.0 / PI << endl;
            tf::quaternionTFToMsg(tf::createQuaternionFromYaw( 8.0*(yaw_P * 1.0 - yaw_I * 0.2)), msg.pose.orientation);
        }
        cout << "controll yaw: " << tf::getYaw(msg.pose.orientation) * 180.0 / PI << endl;
        pub.publish(msg);
    }
}







