#ifndef VEHICLE_H
#define VEHICLE_H
#include <ros/ros.h>
#include <time.h>
#include <iostream>
#include <string>
#include <vector>
#include <boost/variant.hpp>
#include <math.h>
#include <cross_fsm/condition.h>
#include <cross_fsm/query_variable.h>

using namespace std;

class fsm_state
{
protected:

    Condition &crossroad_condition;
    fsm_state* next_state = nullptr;

public:
//    static clock_t startTime_w;
//    static clock_t startTime_f;

    fsm_state(Condition& condition);
    virtual ~fsm_state();
    virtual void action();
    virtual fsm_state* update_state() = 0;
    virtual int turn2int()const = 0;
    int getcrossarea()const;
};




class Buffer : public fsm_state
{
public:
  Buffer(Condition& condition);
  void action();
  fsm_state* update_state();
  int turn2int()const;
};

class KeepLane : public fsm_state
{
public:
    KeepLane(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};

class PrepLeft : public fsm_state
{
public:
    PrepLeft(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};

class PrepRight : public fsm_state
{
public:
    PrepRight(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};

class TurnLeft : public fsm_state
{
public:
    TurnLeft(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};

class TurnRight : public fsm_state
{
public:
    TurnRight(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};

class Stop : public fsm_state
{
public:
    Stop(Condition& condition);
    void action();
    fsm_state* update_state();
    int turn2int()const;

};



#endif // VEHICLE_H
