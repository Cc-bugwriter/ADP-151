#include "cross_fsm/vehicle.h"

/*
 *  interpretation of state
 *    number  vs.  state
 *      1           Buffer
 *      2           Keep Lane
 *      3           Prep Left
 *      4           Prep Right
 *      5           Turn Left
 *      6           Turn Right
 *      7           Stop
 *      8           Change Lane left
 *      9           Change Lane right
 */


fsm_state::fsm_state(Condition& condition): crossroad_condition(condition){}

fsm_state::~fsm_state(){}

void fsm_state::action(){
    //TODO
}

int fsm_state::getcrossarea()const{
    return crossroad_condition.area_recognition();
}


// ===============================
// ========= State Buffer ========
// ===============================

Buffer::Buffer(Condition& condition):fsm_state(condition){}

void Buffer::action(){}

fsm_state* Buffer::update_state()
{    

    cout << "Buffer is updating......" << endl;

    if (crossroad_condition.isBufferToStop())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in Buffer state, next state: KL" << endl;
    }
    else if (crossroad_condition.isBufferToPrepLeft())
    {
        next_state = new PrepLeft(crossroad_condition);
        cout << "now in Buffer state, next state: PrepLeft" << endl;
    }
    else if (crossroad_condition.isBufferToPrepRight())
    {
        next_state = new PrepRight(crossroad_condition);
        cout << "now in Buffer state, next state: PrepRight" << endl;
    }
    else if (crossroad_condition.isBufferToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in Buffer state, next state: Stop" << endl;
    }else {
        cout << "Buffer state not changed" << endl;
        next_state = new Buffer(crossroad_condition);
    }

    return next_state;
}

int Buffer::turn2int()const{
    return 1;
}




// ===============================
// =========== State KL ==========
// ===============================

void KeepLane::KeepLane::action()
{
    void light_off();
}

KeepLane::KeepLane(Condition& condition):fsm_state(condition){}

fsm_state* KeepLane::update_state()
{   
    cout << "State KeepLane is updating......" << endl;

    if (crossroad_condition.isKeepLaneToStop())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in KeepLane state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isKeepLaneToPrepLeft())
    {
        next_state = new PrepLeft(crossroad_condition);
        cout << "now in KeepLane state, next state: Prep change left" << endl;
    }
    else if (crossroad_condition.isKeepLaneToPrepRight())
    {
        next_state = new PrepRight(crossroad_condition);
        cout << "now in KeepLane state, next state: Prep change right" << endl;
    }
    else if (crossroad_condition.isKeepLaneToBuffer())
    {
        next_state = new Buffer(crossroad_condition);
        cout << "now in KeepLane state, next state: Buffer" << endl;
    }
    else if (crossroad_condition.isKeepLaneReflexive())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in KeepLane state, next state: KeepLane" << endl;
    }else {
        cout << "KeepLane state not changed" << endl;
        next_state = new KeepLane(crossroad_condition);
    }

    return next_state;
}

int KeepLane::turn2int()const{
    return 2;
}



// ===============================
// =========== State PCL =========
// ===============================

void PrepLeft::action()
{
    // Activating the signal light and computing the possibility of turning right/left
    void light_on();
    void computePossibility();
}

PrepLeft::PrepLeft(Condition& condition):fsm_state(condition){}

fsm_state* PrepLeft::update_state()
{
    cout << "State Prep Change Left is updating......" << endl;

    if (crossroad_condition.isPrepLeftToStop())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in PrepLeft state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isPrepLeftToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in PrepLeft state, next state: KeepLane" << endl;
    }
    else if (crossroad_condition.isPrepLeftToTurnLeft())
    {
        next_state = new TurnLeft(crossroad_condition);
        cout << "now in PrepLeft state, next state: TurnLeft" << endl;
    }else {
        cout << "PrepLeft state not changed" << endl;
        next_state = new PrepLeft(crossroad_condition);
    }


    return next_state;
}

int PrepLeft::turn2int()const{
    return 3;
}

// ===============================
// =========== State PCR =========
// ===============================

void PrepRight::action()
{
    // Activating the signal light and computing the possibility of turning right/left

    void light_on();
    void computePossibility();
}

PrepRight::PrepRight(Condition& condition):fsm_state(condition){}

fsm_state* PrepRight::update_state()
{

    cout << "State Prep Change Right is updating......" << endl;

    if (crossroad_condition.isPrepRightToStop())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in PrepRight state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isPrepRightToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in PrepRight state, next state: KeepLane" << endl;
    }
    else if (crossroad_condition.isPrepRightToTurnRight())
    {
        next_state = new TurnRight(crossroad_condition);
        cout << "now in PrepRight state, next state: TurnRight" << endl;
    }else {
        cout << "PrepRight state not changed" << endl;
        next_state = new PrepRight(crossroad_condition);
    }

    return next_state;
}

int PrepRight::turn2int()const{
    return 4;
}


// ===============================
// =========== State LCL =========
// ===============================

void TurnLeft::action()
{

}

TurnLeft::TurnLeft(Condition& condition):fsm_state(condition){}

fsm_state* TurnLeft::update_state()
{

    cout << "State Turn Left is updating......" << endl;

    if (crossroad_condition.isTurnLeftToStop())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in TurnLeft state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isTurnLeftToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in TurnLeft state, next state: KeepLane" << endl;
    }else {
        cout << "TurnLeft state not changed" << endl;
        next_state = new TurnLeft(crossroad_condition);
    }

    return next_state;
}

int TurnLeft::turn2int()const{
    return 5;
}


// ===============================
// =========== State LCR =========
// ===============================

void TurnRight::action()
{

}

TurnRight::TurnRight(Condition& condition):fsm_state(condition){}

fsm_state* TurnRight::update_state()
{

    cout << "State Turn Right is updating......" << endl;

    if (crossroad_condition.isTurnRightToStop())
    {
        next_state = new Stop(crossroad_condition);

        cout << "now in TurnRight state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isTurnRightToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in TurnRight state, next state: KeepLane" << endl;
    }else {
        cout << "TurnRight state not changed" << endl;
        next_state = new TurnRight(crossroad_condition);
    }

    return next_state;
}

int TurnRight::turn2int()const{
    return 6;
}


// ===============================
// ========== State Stop =========
// ===============================
void Stop::action()
{
    void emergency();
}

Stop::Stop(Condition& condition):fsm_state(condition){}

fsm_state* Stop::update_state()
{

    cout << "State Stop is updating......" << endl;

    if (crossroad_condition.isStopReflexive())
    {
        next_state = new Stop(crossroad_condition);
        cout << "now in Stop state, next state: Stop" << endl;
    }
    else if (crossroad_condition.isStopToKeepLane())
    {
        next_state = new KeepLane(crossroad_condition);
        cout << "now in Stop state, next state: KeepLane" << endl;
    }
    else if (crossroad_condition.isStopToPrepLeft())
    {
        next_state = new PrepLeft(crossroad_condition);
        cout << "now in Stop state, next state: PrepLeft" << endl;
    }
    else if (crossroad_condition.isStopToPrepRight())
    {
        next_state = new PrepRight(crossroad_condition);
        cout << "now in Stop state, next state: PrepRight" << endl;
    }else {
        cout << "Stop state not changed" << endl;
        next_state = new Stop(crossroad_condition);
    }

    return next_state;
}

int Stop::turn2int()const{
    return 7;
}

