
#ifndef VEHICLE_H
#define VEHICLE_H
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string>
#include <vector>
#include <boost/variant.hpp>
#include <math.h>
#include <straight_fsm/query_variable.h>
#include <tf/tf.h>
#include<vaafo_msgs/State.h>


using namespace std;

class fsm_state
{
public:
  float const ttc = 9999;      // time to collision
  float const lc = 9999;       // the distance to collision
  float const STOPLINE_far = 0;  // changing lane actions are not allowed when distance between ego car and stopline is smaller than this
  float const STOPLINE_near = 2;  // GoStraight can not deal with the situation when the distance between ego car and stopline is smaller than this
  float const TIME_interval = 200;  // unit:nsec, the time that must wait because of the previous failure
  float const TIME_wait = 300;  // unit:, nsec, the maximum waiting time, once waiting longer than TIME_wait, report failure
  float const TIME_change = 5;     // expected time of changing lanes
  float const GAP_vertical = 5;  // the minimum vertical distance between the left/right frontal car and the left/right back car in target lane
  float const GAP_lateral = 3.5;  // the minimum lateral distance between ego car and the calculated gap in the other lane
  float const criterium = 0.5; // the criterium for cost function


  static bool turnLeft_Motiv;// not defined
  static bool turnRight_Motiv;// not defined

 // bool collision;

  double acc_brake_max=10.0; // the max acceleration to brake
  double t_refresh=0.1; // the time to refresh
  double const s_emb_min=2.0; // the mindest distance between the egocar and the front car
  float stopline_coord=9999; // not defined
  float stopline_last_coord;
  int soll_lane_id;  // not defined
  int ego_lane_id;

  //Ego_coord ego_coord;
  vaafo_msgs::CarInfo egoCar;
  vaafo_msgs::CarInfo egoCar_new;
  vaafo_msgs::CarInfo egoCar_target;

  // ego car transformation
  void worldtofrenet(vaafo_msgs::CarInfo egoCar);

  std::vector<vaafo_msgs::DetectedObject> objects;
  std::vector<vaafo_msgs::SensorObjectInfo> otherObjects;

  vaafo_msgs::SensorObjectInfo front_Object;
  vaafo_msgs::SensorObjectInfo f_Object;
  vaafo_msgs::SensorObjectInfo fl_Object;
  vaafo_msgs::SensorObjectInfo fr_Object;
  vaafo_msgs::SensorObjectInfo bl_Object;
  vaafo_msgs::SensorObjectInfo br_Object;


  float gap_leftLane;
  float gap_llateralDis;
  float gap_rightLane;
  float gap_rlateralDis;
  bool collision;


  fsm_state(queryVariable qV);
  ~fsm_state();
  virtual void action();
  bool collision_check(vaafo_msgs::SensorObjectInfo otherobject);
  bool collision_all(std::vector<vaafo_msgs::SensorObjectInfo> otherobjects);


  void set_target_ego_velocity(int s);
  void set_target_ego_position(int s);
  virtual uint8_t turn2int()const = 0;
  virtual fsm_state* update_state(queryVariable qV) = 0;

};

class Buffer : public fsm_state
{
public:
  using fsm_state::fsm_state;
  void action();  
  fsm_state* update_state(queryVariable qV);
  uint8_t turn2int()const ;

private:
  bool isBufferToKeepLane();
  bool isBufferToPrepLeft();
  bool isBufferToPrepRight();
  bool isBufferToStop();
};

class KL : public fsm_state
{
public:
  using fsm_state::fsm_state;
  void action();
  fsm_state* update_state(queryVariable qV);
  uint8_t turn2int()const ;

private:
  //float stopline_coord;  // original stoplines[next], the coordinate of the next (nearest) stopline
  //float ego_coord;       // original ego.position, the coordinate of the ego car
  bool isKeepLaneToBuffer();
  bool isKeepLaneToPrepLeft();
  bool isKeepLaneToPrepRight();
  bool isKeepLaneToStop();
  bool cost_function();
};

class PCL : public fsm_state
{
public:

  void lignt_on_left();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);
  uint8_t turn2int()const ;

private:
  bool isPrepLeftToKeepLane();
  bool isPrepLeftToTurnLeft();
  bool isPrepLeftToStop();
};

// class PCR :public PCL,fsm_state
class PCR : public fsm_state
{

public:

    void light_on_right();
    using fsm_state::fsm_state;
    fsm_state* update_state(queryVariable qV);
    uint8_t turn2int()const ;

private:
  bool isPrepRightToKeepLane();
  bool isPrepLeftToTurnRight();
  bool isPrepRightToStop();
};

class LCL : public fsm_state
{
public:
  void action();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);
  uint8_t turn2int()const ;

private:
  bool isTurnLeftToKeepLane();
  bool isTurnLeftToStop();
};

class LCR : public fsm_state
{
public:
  void action();
  using fsm_state::fsm_state;
  fsm_state* update_state(queryVariable qV);
  uint8_t turn2int()const ;

private:
  bool isTurnRightToKeepLane();
  bool isTurnRightToStop();
};

class Stop : public fsm_state
{
public:
    using fsm_state::fsm_state;
    void action();
    fsm_state* update_state(queryVariable qV);
    uint8_t turn2int()const ;

private:
  bool isStopToKeepLane();
};


#endif // VEHICLE_H
