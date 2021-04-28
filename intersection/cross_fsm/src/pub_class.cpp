#include "ros/ros.h"
#include "cross_fsm/pub_class.h"
#include <numeric>

pub::pub(ros::NodeHandle& nh) {
    chatter_pubstate = nh.advertise<vaafo_msgs::State>("behavior_state", 10);

    // generation Frenet reference line
    readMap(x_cord, y_cord);
    calcS(frenet_s);
}

void pub::pubState(const fsm_state* fs, const queryVariable& qv) {
    vaafo_msgs::State pub_state;
    pub_state.state_id = fs->turn2int();
    pub_state.header = qv.getEgocar().header;
    pub_state.target.header = qv.getEgocar().header;
    pub_state.target.acceleration = goalacceleration();
    pub_state.target.twist = goalvelocity(fs, qv);
    pub_state.target.pose = goalposition(fs, qv);
    pub_state.target.s_d = goalfrenet(fs, qv);
    pub_state.ego_car = qv.getEgocar();
    // compensation vehicle coordinate offset
    geometry_msgs::Point offset;
    offset.x = 3.53;
    pub_state.ego_car.pose.position.x = qv.MapPositionX(offset);
    pub_state.ego_car.pose.position.y = qv.MapPositionY(offset);
    pub_state.ego_car.acceleration.linear.x = qv.EgoAccelerationX();
    pub_state.ego_car.acceleration.linear.y = qv.EgoAccelerationY();
    pub_state.obstacle = qv.getDecObject();

//    ROS_INFO("publish target in map coordinate");
//    std::cout<<"pub:target_position_x: "<< pub_state.target.pose.position.x <<std::endl;
//    std::cout<<"pub:target_position_y: "<< pub_state.target.pose.position.y <<std::endl;
//    std::cout<<"pub:target_velocity_x: "<< pub_state.target.twist.linear.x <<std::endl;
//    std::cout<<"pub:target_velocity_y: "<< pub_state.target.twist.linear.y<<std::endl;

    chatter_pubstate.publish(pub_state);
}

geometry_msgs::Pose pub::goalposition(const fsm_state* fs, const queryVariable& qv) {
    geometry_msgs::Pose goalPose;
    geometry_msgs::Point relPose;
    double pose_x_motion = qv.getNearFront() == -1 ? 125.0 / 9.0 * t_cruise : t_cruise * (qv.EgoVelocityX() + 0.5 * qv.RelativVelocityX(qv.getDecObject().velocity));
    double pose_y_motion = qv.getNearFront() == -1 ? 0 * t_cruise : t_cruise * (qv.EgoVelocityY() + 0.5 * qv.RelativVelocityY(qv.getDecObject().velocity));
    double pose_x_obj = qv.RelativPositionX(qv.getDecObject().pose.position);
    double pose_y_obj = qv.RelativPositionY(qv.getDecObject().pose.position);
    double pose_x_taffic = qv.RelativPositionX(qv.getTrafficLight().pose.position) - 7.0;
    double pose_y_taffic = qv.RelativPositionY(qv.getTrafficLight().pose.position);

    // start problem
    if (pose_x_obj > 125.0 / 9.0 * t_cruise) {
        pose_x_motion = 125.0 / 9.0 * t_cruise;
        pose_y_motion =  0 * t_cruise;
    }

//    ROS_INFO("relative distance");
//    std::cout<<"target:target index: "<< qv.getNearFront() <<std::endl;
//    std::cout<<"target:free_pose_x: "<< pose_x_motion <<std::endl;
//    std::cout<<"target:free_pose_y: "<< pose_y_motion <<std::endl;
//    std::cout<<"target:rel_pose_x: "<< pose_x_obj <<std::endl;
//    std::cout<<"target:rel_pose_y: "<< pose_y_obj <<std::endl;

    if(fs->turn2int() != 7) {
        // not stop state
        if (sqrt(pow(pose_x_motion, 2) + sqrt(pow(pose_y_motion, 2))) <
                sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
            relPose.x = pose_x_motion;
            relPose.y = pose_y_motion;
        } else {
            // TODO
            relPose.x = pose_x_obj - 4.0;
            relPose.y = pose_y_obj;
        }
    }
    else {
        // stop state
        if(fs->getcrossarea() == 0 || fs->getcrossarea() == 1) {
            //before the crossroad
            if (sqrt(pow(pose_x_taffic, 2) + sqrt(pow(pose_y_taffic, 2))) <
                    sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
                relPose.x = pose_x_taffic;
                relPose.y = pose_y_taffic;
            } else {
                relPose.x = pose_x_obj - 4.0;
                relPose.y = pose_y_obj;
            }
        }
        else {
            //in the crossroad
            if (sqrt(pow(pose_x_motion, 2) + sqrt(pow(pose_y_motion, 2))) <
                    sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
                relPose.x = pose_x_motion;
                relPose.y = pose_y_motion;
            } else {
                // TODO
                relPose.x = pose_x_obj - 4.0;
                relPose.y = pose_y_obj;
            }
        }
    }

    // valid check
    if (relPose.x > 0) {
        goalPose.position.x = qv.MapPositionX(relPose);
        goalPose.position.y = qv.MapPositionY(relPose);
    }else {
        goalPose.position = qv.getEgocar().pose.position;
    }

    return goalPose;
}

geometry_msgs::Twist pub::goalvelocity(const fsm_state* fs, const queryVariable& qv) {
    geometry_msgs::Twist goalVel;
    geometry_msgs::Twist limitVel;
    geometry_msgs::Twist objVel;

    limitVel.linear.x = 125.0 / 9.0 - qv.EgoVelocityX();
    limitVel.linear.y = 0.0 - qv.EgoVelocityY();
    objVel.linear.x = qv.RelativVelocityX(qv.getDecObject().velocity);
    objVel.linear.y = qv.RelativVelocityY(qv.getDecObject().velocity);

//    ROS_INFO("relative velocity");
//    std::cout<<"target:free_velocity_x: "<< limitVel.linear.x <<std::endl;
//    std::cout<<"target:free_velocity_y: "<< limitVel.linear.y <<std::endl;
//    std::cout<<"target:rel_velocity_x: "<< objVel.linear.x <<std::endl;
//    std::cout<<"target:rel_velocity_y: "<< objVel.linear.y <<std::endl;

    if(fs->turn2int() != 7){
        double pose_x_obj = qv.RelativPositionX(qv.getDecObject().pose.position);
        double pose_y_obj = qv.RelativPositionY(qv.getDecObject().pose.position);

        if (sqrt(pow(limitVel.linear.x, 2) + pow(limitVel.linear.y, 2)) <
                sqrt(pow(objVel.linear.x, 2) + pow(objVel.linear.y, 2))) {
            goalVel.linear.x = qv.MapVelocityX(limitVel);
            goalVel.linear.y = qv.MapVelocityY(limitVel);
        } else {
            if (sqrt(pow(pose_x_obj, 2) + pow(pose_y_obj, 2)) < 125.0 / 9.0 * t_cruise) {
                goalVel.linear.x = qv.MapVelocityX(objVel);
                goalVel.linear.y = qv.MapVelocityY(objVel);
            } else {
                goalVel.linear.x = qv.MapVelocityX(limitVel);
                goalVel.linear.y = qv.MapVelocityY(limitVel);
            }

        }
    }
    else {
        // stop state
        goalVel.linear.x = 0.0;
        goalVel.linear.y = 0.0;
        goalVel.linear.z = 0.0;
    }
    return goalVel;
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

void pub::calcS(std::vector<double> &s) {
    std::vector<double> dx(x_cord.size());
    std::vector<double> dy(y_cord.size());
    std::adjacent_difference(x_cord.begin(), x_cord.end(), dx.begin());
    dx.erase(dx.begin());
    std::adjacent_difference(y_cord.begin(), y_cord.end(), dy.begin());
    dy.erase(dy.begin());
    double cum_sum = 0.0;
    s.push_back(cum_sum);
    for (size_t i = 0; i < x_cord.size()-1; i++) {
        cum_sum += sqrt((std::pow(dx[i],2)+std::pow(dy[i],2)));
        s.push_back(cum_sum);
    }
    s.erase(unique(s.begin(),s.end()),s.end());
}

/*
 * ------------------------------------------additional block------------------------------------
 * transformation the motion target in frenet coordinate
 */
geometry_msgs::Point pub::goalfrenet(const fsm_state* fs, const queryVariable& qv) {
    geometry_msgs::Point goalFrenet;
    geometry_msgs::Point relPose;
    double pose_x_motion = qv.getNearFront() == -1 ? 125.0 / 9.0 * t_cruise : t_cruise * (qv.EgoVelocityX() + 0.5 * qv.RelativVelocityX(qv.getDecObject().velocity));
    double pose_y_motion = qv.getNearFront() == -1 ? 0 * t_cruise : t_cruise * (qv.EgoVelocityY() + 0.5 * qv.RelativVelocityY(qv.getDecObject().velocity));
    double pose_x_obj = qv.RelativPositionX(qv.getDecObject().pose.position);
    double pose_y_obj = qv.RelativPositionY(qv.getDecObject().pose.position);
    double pose_x_taffic = qv.RelativPositionX(qv.getTrafficLight().pose.position) - 7.0;
    double pose_y_taffic = qv.RelativPositionY(qv.getTrafficLight().pose.position);

    // start problem
    if (pose_x_obj > 125.0 / 9.0 * t_cruise) {
        pose_x_motion = 125.0 / 9.0 * t_cruise;
        pose_y_motion =  0 * t_cruise;
    }

    if(fs->turn2int() != 7) {
        // not stop state
        if (sqrt(pow(pose_x_motion, 2) + sqrt(pow(pose_y_motion, 2))) <
                sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
            relPose.x = pose_x_motion;
            relPose.y = pose_y_motion;
        } else {
            // TODO
            relPose.x = pose_x_obj;
            relPose.y = pose_y_obj;
        }
    }
    else {
        // stop state
        if(fs->getcrossarea() == 0 || fs->getcrossarea() == 1) {
            //before the crossroad
            if (sqrt(pow(pose_x_taffic, 2) + sqrt(pow(pose_y_taffic, 2))) <
                    sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
                relPose.x = pose_x_taffic;
                relPose.y = pose_y_taffic;
            } else {
                relPose.x = pose_x_obj;
                relPose.y = pose_y_obj;
            }
        }
        else {
            //in the crossroad
            if (sqrt(pow(pose_x_motion, 2) + sqrt(pow(pose_y_motion, 2))) <
                    sqrt(pow(pose_x_obj, 2) + sqrt(pow(pose_y_obj, 2)))) {
                relPose.x = pose_x_motion;
                relPose.y = pose_y_motion;
            } else {
                // TODO
                relPose.x = pose_x_obj;
                relPose.y = pose_y_obj;
            }
        }
    }

    // valid check
    if (relPose.x > 4) {
        goalFrenet.x = qv.MapPositionX(relPose);
        goalFrenet.y = qv.MapPositionY(relPose);
    }else {
        goalFrenet = qv.getEgocar().pose.position;
    }

    std::vector<double> dis;
    for (size_t i = 0; i < x_cord.size(); i++) {
            //calculate the distance between cart point and reference line
            dis.push_back(sqrt(std::pow(x_cord[i] - goalFrenet.x, 2))+std::pow(y_cord[i] - goalFrenet.y, 2));
        }
        //find the index of the reference point
        size_t idx = std::min_element(dis.begin(),dis.end()) - dis.begin(); //under <algorithm>
    goalFrenet.x = frenet_s[idx];

    return goalFrenet;
}


