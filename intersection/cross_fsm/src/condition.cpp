#include "cross_fsm/condition.h"

// collision check required parameters
double Condition::acc_brake_max = 10;               // maximal deceleration,  m / s ^ 2
double Condition::acc_brake_soft_max = 3.5;         // maximal soft deceleration, m / s ^ 2
double Condition::acc_brake_soft_min = 2.5;         // minimal soft deceleration, m / s ^ 2
double Condition::t_refresh = 0.1;                  // refresh time interval, s
double Condition::emb_min = 4;                      // minimal emergency brake distance in low speed state, m
double Condition::emb_brake_proportion = 0.5;       // emergency braking proportion in collision chech, [0, 1]

double Condition::crosswalk_pose_x_near = -1;
double Condition::crosswalk_pose_x_far = 2;
double Condition::slow_approach = 4;

double Condition::radar_offset = 3.53;
double Condition::carrosserie = 5.466;

Condition::Condition(queryVariable& qV): queryVar(qV){};

Condition::~Condition(){};

