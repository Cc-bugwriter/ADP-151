#include <straight_fsm/vehicle.h>
using namespace std;


bool fsm_state::turnLeft_Motiv = false;
bool fsm_state::turnRight_Motiv = false;

fsm_state::fsm_state(queryVariable qV)
{

    // get the information of the objects around ego car
    qV.get_otherObjects();
    front_Object = qV.otherObjects[0];
    fl_Object = qV.otherObjects[1];
    bl_Object = qV.otherObjects[2];
    fr_Object = qV.otherObjects[3];
    br_Object = qV.otherObjects[4];


    ego_lane_id = qV.ego_lane_id;
    egoCar = qV.egoCar;
    objects = qV.objects;
    worldtofrenet(egoCar);

    // calculate the gap in the left lane and relative distance between the ego car and the "gap"
    gap_leftLane = (fl_Object.velocity.linear.x - bl_Object.velocity.linear.x) * TIME_change +
            (fl_Object.pose.position.x - bl_Object.pose.position.x);


    gap_llateralDis = min(abs(fl_Object.pose.position.x + fl_Object.velocity.linear.x* TIME_change),
                          abs(bl_Object.pose.position.x + bl_Object.velocity.linear.x*TIME_change));//

    // calculate the gap in the right lane and relative distance between the ego car and the "gap"
    gap_rightLane = (fr_Object.velocity.linear.x - br_Object.velocity.linear.x) * TIME_change +
            (fr_Object.pose.position.x - br_Object.pose.position.x);
    gap_rlateralDis = min(abs(fr_Object.pose.position.x + fr_Object.velocity.linear.x* TIME_change),
                          abs(br_Object.pose.position.x + br_Object.velocity.linear.x*TIME_change));


    collision= collision_all(qV.lidar_objects);


}

/*
 * transform the postion and velocity of ego car from world coordination to frenet coordination
 * reassign the new position and velocity to the egoCar_new
 * */

void fsm_state::worldtofrenet(vaafo_msgs::CarInfo egoCar){



    tf::Quaternion quat;
    tf::quaternionMsgToTF(egoCar.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double velocity_x =  egoCar.twist.linear.x*cos(yaw)+egoCar.twist.linear.y*sin(yaw);
    double velocity_y =  - egoCar.twist.linear.x*sin(yaw)+ egoCar.twist.linear.y*cos(yaw);

    double pose_x =  egoCar.pose.position.x*cos(yaw)+egoCar.pose.position.y*sin(yaw) + 909.34;
    double pose_y =  - egoCar.pose.position.x*sin(yaw)+ egoCar.pose.position.y*cos(yaw);

    egoCar_new.twist.linear.x = velocity_x;
    egoCar_new.twist.linear.y = velocity_y;
    egoCar_new.pose.position.x = pose_x;
    egoCar_new.pose.position.y = 0.0;

}

/*
 * set the target velocity of egoCar
 * the default velocity of ego car is 50 km/h(13.88 m/s)
 * */


void fsm_state::set_target_ego_velocity(int s){

    // set the velocity same as the ego Car
    if (s==0){
        egoCar_target.twist.linear.x = egoCar_new.twist.linear.x;
        egoCar_target.twist.linear.y = egoCar_new.twist.linear.y;


    }else if(s==1){
        // the ego car begin to stop
        egoCar_target.twist.linear.x = 0.0;
        egoCar_target.twist.linear.y = 0.0;



    }else{
        // the ego car has the same velocity as the front car

        if (front_Object.velocity.linear.x > 900){
            egoCar_target.twist.linear.x = 13.88;
            egoCar_target.twist.linear.y = 0;

        }else{
            egoCar_target.twist.linear.x = egoCar_new.twist.linear.x+front_Object.velocity.linear.x;
            egoCar_target.twist.linear.y = egoCar_new.twist.linear.y+front_Object.velocity.linear.y;
        }

    }


}

/*
 * set the target position of egoCar to egoCar_target
 *
 * */

void fsm_state::set_target_ego_position(int s){
    // in x direction: position = old_position +(v_old+v_new)/2* 0.1

    egoCar_target.pose.position.x = (egoCar_new.twist.linear.x+ egoCar_target.twist.linear.x)/2.0*0.1
            + egoCar_new.pose.position.x;

    if (s==0){
        // the ego car will not change the lane
        egoCar_target.pose.position.y = egoCar_new.pose.position.y;
    }
    else if(s==1){
        // the ego car will change lane left
        egoCar_target.pose.position.y = egoCar_new.pose.position.y + 3.5;
    }else{
        // the ego car will change lane right
        egoCar_target.pose.position.y = egoCar_new.pose.position.y - 3.5;
    }

}



void fsm_state::action(){
  //TODO
}

/*
 * check if the ego car will meet the collisiton
 * */

bool fsm_state::collision_check(vaafo_msgs::SensorObjectInfo otherobject){

    bool Collision;
    double entity_velocity_x = otherobject.velocity.linear.x;
    double entity_velocity_y = 0;
    // ego car velocity

    double ego_velocity_x =  egoCar_new.twist.linear.x;


    double collision_boundary_x = pow(entity_velocity_x,2)/acc_brake_max+
             ego_velocity_x*t_refresh + s_emb_min;

    double collision_boundary_y = 1.5;
    double entity_position_x = otherobject.pose.position.x;
    double entity_position_y = otherobject.pose.position.y;


    if(entity_velocity_x <0){

        // check collision for slower obstacle
        if ((entity_position_x>0 && entity_position_x<collision_boundary_x)&& (abs(entity_position_y)<collision_boundary_y)){
            Collision = true;
        }else {
            if (entity_velocity_y==0.0){
                Collision=false;
            }else {
                if(entity_position_x>0 && abs(entity_position_y)>1.5){
                    // traffic participant on both sides (right and left)
                    double t_cut_in = abs(entity_position_y)/entity_velocity_y;
                    // predict the obstacle position after cut in
                    double prediction_pose_x = entity_position_x + t_cut_in*entity_velocity_x;

                    if(prediction_pose_x>0.0 && prediction_pose_x< collision_boundary_x){
                        Collision = true;
                    }else {
                        Collision = false;
                    }
                }else {
                  Collision=false;
                }
            }
        }
    }else {
        Collision=false;
    }
    return Collision;
}



bool fsm_state::collision_all(std::vector<vaafo_msgs::SensorObjectInfo> otherobjects){

    int num = int(otherobjects.size());
    std::cout<<"number of the objects: "<<num<<std::endl;
    bool collision;
    for(int i=0; i<num; i++){
        collision = collision_check(otherobjects.at(i));
        if(collision==true)
            return true;
    }
    return false;

}



// ===============================
// ========= State Buffer ========
// ===============================

void Buffer::action(){
  // accept command from the previous FSM || return
}


fsm_state* Buffer::update_state(queryVariable qV)
{
  cout << "Buffer is updating......" << endl;
  fsm_state* next_state = NULL; 
  // next state option: s1, s17, s18
  if (isBufferToKeepLane())
  { // next state: Keep Lane
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    cout << "next state: KL" << endl;
    next_state = new KL(qV);
  }
  else if (isBufferToPrepLeft())
  {
   // next stste: Prep change Left;
    cout << "next state: PCL" << endl;
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new PCL(qV);
  }
  else if (isBufferToPrepRight())
  { // next State: Prep change right
    cout << "next state: PCR" << endl;
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new PCR(qV);

  }
  else if (isBufferToStop())
  { // next state: Stop
    cout << "next state: Stop" << endl;
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new Stop(qV);
  }
  else {
	//next state: this state
    cout << "next state: this state" << endl;
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new Buffer(qV);
  }
  return next_state;
}

uint8_t Buffer::turn2int()const{
    return 1;
}

bool Buffer::isBufferToKeepLane(){
    // from buffer to keepLane

   bool condition = collision== false && turnLeft_Motiv == false && turnRight_Motiv == false;

    return condition;
}

bool Buffer::isBufferToPrepLeft()
{
  bool condition = collision == false && turnLeft_Motiv == true;
  return condition;
}

bool Buffer::isBufferToPrepRight()
{
  bool condition = collision == false && turnRight_Motiv == true;
  return condition;
}

bool Buffer::isBufferToStop()
{
  bool condition = collision == true;
  return condition;
}



// ===============================
// =========== State KL ==========
// ===============================

void KL::action()
{
  // 是否返回信号
  void keepdrive();
  void acceleration();
  void deccleration();
}

/*
 * decide if the ego car overtake
 * if overtake, the result will return true
 * */

bool KL::cost_function()
{

    // need to be read from the path message
    // already in the fast path, don't need to overtake
    int changed_id;
    if (ego_lane_id==-2){
        changed_id =-1;
    }
    else{
        changed_id = 1;
    }

    int cost1;
    float cost2;
    float cost3;
    float w1 = 1.0;
    float w2 = 1.0;
    float w3 = 1.0;
    float path_length = stopline_coord-stopline_last_coord;// the whole length of this straight path
    // cost1 function for lane change
    if (soll_lane_id == changed_id){
        cost1 = 0;
    }else {
        cost1 = 1;
    };
    // cost2 function for speed
    cost2 = (front_Object.velocity.linear.x + egoCar_new.twist.linear.x)/egoCar_new.twist.linear.x;
    cout<<"the velocity of front car:"<<front_Object.velocity.linear.x + egoCar_new.twist.linear.x<<endl;
    cout<<"the ratio velocity of car:"<<cost2<<endl;

    double distance = front_Object.pose.position.x;

    if (cost2<criterium){
        cout<<"the distance is:"<<distance<<endl;
    }

    // cost3 function for distance
    // stopline here is in the geodetic coordination
    // ratio or relative length
    cost3 = (egoCar_new.pose.position.x - stopline_last_coord)/path_length;

    //float cost = w1*cost1+ w2*cost2 + w3*cost3;
    float cost =w2*cost2;

    bool judge = (cost <= criterium) && (distance < 30);
    return judge;
}

// TODO：stoplines[next], costfunction
fsm_state* KL::update_state(queryVariable qV)
{
  cout << "State KeepLane is updating......" << endl;
  fsm_state* next_state = NULL;
  // transition condition option: s2, s3, s4, s15
  if (isKeepLaneToBuffer())
  {

    // transition condition: s2, next state: Buffer
    set_target_ego_velocity(1);
    set_target_ego_position(0);
    next_state = new Buffer(qV);
    cout << "next state: Buffer" << endl;
  }
  else if (isKeepLaneToPrepLeft())
  {  // transition condition: s3, next state: Prep change left

    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new PCL(qV);
    cout << "next state: Prep change left" << endl;
   // ros::shutdown();
  }
  else if (isKeepLaneToPrepRight())
  {

    // transition condition: s4, next state: Prep change right
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new PCR(qV);
    cout << "next state: Prep change right" << endl;
  }
  else if (isKeepLaneToStop()){
    // transition condition: s15, next state: Stop


      set_target_ego_velocity(1);
      set_target_ego_position(0);
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
      //ros::shutdown();
  }
  else
  {
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  return next_state;
}
uint8_t KL::turn2int()const{
    return 2;
}

bool KL::isKeepLaneToBuffer()
{
  bool condition = ((stopline_coord - egoCar_new.pose.position.x) < STOPLINE_near) && collision == false;
  return condition;
}

bool KL::isKeepLaneToPrepLeft()
{
  // add cost function here

    bool cost = cost_function();
    bool condition = collision == false && ((stopline_coord - egoCar_new.pose.position.x) > STOPLINE_far) &&
                       ((turnLeft_Motiv == true) || cost == true);
  return condition;
}

bool KL::isKeepLaneToPrepRight()
{
  bool condition = collision == false && ((stopline_coord - egoCar_new.pose.position.x) > STOPLINE_far) &&
                   turnRight_Motiv == true;
  return condition;
}

bool KL::isKeepLaneToStop()
{
  bool condition = (collision == true);
  return condition;
}


// ===============================
// =========== State PCL =========
// ===============================



fsm_state* PCL::update_state(queryVariable qV)
{
    cout << "State Prep Change Left is updating......" << endl;

    // transition condition option: s5, s7, s13

  fsm_state* next_state = NULL;
  if (isPrepLeftToKeepLane())
  {

    // transition condition: s5, next state: Keep Lane
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;

  }
  else if (isPrepLeftToTurnLeft())
  {  // transition condition: s7, next state: Lane change left

      set_target_ego_velocity(0);
      set_target_ego_position(1);
      next_state = new LCL(qV);
      cout << "next state: LCL" << endl;
      ros::shutdown();
  }
  else if (isPrepLeftToStop())
  {  // transition condition: s13, next state: Stop

      set_target_ego_velocity(1);
      set_target_ego_position(0);
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
      ros::shutdown();
  }
  else{
      set_target_ego_velocity(0);
      set_target_ego_position(0);
      next_state = new PCL(qV);
      cout << "next state: PCL" << endl;
      ros::shutdown();
  }
  return next_state;
}

uint8_t PCL::turn2int()const{
    return 3;
}

bool PCL::isPrepLeftToKeepLane()
{
    bool condition = (gap_leftLane < GAP_vertical) || (gap_llateralDis < GAP_lateral) && collision == false;
  return condition;
}

bool PCL::isPrepLeftToTurnLeft()
{

  bool condition = (gap_leftLane >= GAP_vertical) && (gap_llateralDis >= GAP_lateral) && collision == false;
  return condition;
}

bool PCL::isPrepLeftToStop()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// =========== State PCR =========
// ===============================




fsm_state* PCR::update_state(queryVariable qV)
{

  cout << "State Prep Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (isPrepRightToKeepLane())
  {

    // transition condition: s6, next state: Keep Lane
    set_target_ego_velocity(0);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (isPrepLeftToTurnRight())
  {  // transition condition: s8, next state: Lane change Right

      set_target_ego_velocity(0);
      set_target_ego_position(2);
      next_state = new LCR(qV);
      cout << "next state: LCR" << endl;
  }
  else if (isPrepRightToStop())
  {  // transition condition: s14, next state: Stop

      set_target_ego_velocity(1);
      set_target_ego_position(0);
      next_state = new Stop(qV);
      cout << "next state: Stop" << endl;
  }
  else{
      set_target_ego_velocity(0);
      set_target_ego_position(0);
      next_state = new PCR(qV);
      cout << "next state: PCR" << endl;
  }
  return next_state;
}

uint8_t PCR::turn2int()const{
    return 4;
}

bool PCR::isPrepRightToKeepLane()
{
  bool condition = collision == false;
  return condition;
}

bool PCR::isPrepLeftToTurnRight()
{
  //float gap_rightLane = (fr_speed - br_speed) * TIME_change + (fr_position - br_position);
  bool condition = (gap_rightLane >= GAP_vertical) && (gap_rlateralDis >= GAP_lateral) && collision == false;
  return condition;
}

bool PCR::isPrepRightToStop()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// =========== State LCL =========
// ===============================

void LCL::action()
{
  void turnLeft();
  bool turnleft_success();
}

fsm_state* LCL::update_state(queryVariable qV)
{
  // transition condition option: s9, s11
  cout << "State Lane Change Left is updating......" << endl;
  fsm_state* next_state = NULL;
  if (isTurnLeftToKeepLane())
  {
    // transition condition: s9, next state: Keep Lane
    set_target_ego_velocity(2);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (isTurnLeftToStop())
  {
    set_target_ego_velocity(1);
    set_target_ego_position(0);
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

uint8_t LCL::turn2int()const{
    return 5;
}

bool LCL::isTurnLeftToKeepLane()
{
  bool condition = soll_lane_id == ego_lane_id && collision == false;
  return condition;
}

bool LCL::isTurnLeftToStop()
{
  bool condition = collision == true;
  return condition;
}


// ===============================
// =========== State LCR =========
// ===============================

void LCR::action()
{
  void turnRight();
  bool turnright_success();
}

fsm_state* LCR::update_state(queryVariable qV)
{
  action();
  // transition condition option: s10, s12
  cout << "State Lane Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (isTurnRightToKeepLane())
  {
    set_target_ego_velocity(2);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else if (isTurnRightToStop())
  {
    set_target_ego_velocity(1);
    set_target_ego_position(0);
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

uint8_t LCR::turn2int()const{
    return 6;
}


bool LCR::isTurnRightToKeepLane()
{
  bool condition = soll_lane_id == ego_lane_id && collision == false;
  return condition;
}

bool LCR::isTurnRightToStop()
{
  bool condition = collision == true;
  return condition;
}

// ===============================
// ========== State Stop =========
// ===============================
void Stop::action()
{
  // Emergency stop and requesting for a new Trajectory
  void emergency();
}

fsm_state* Stop::update_state(queryVariable qV)
{
  action();
  // transition condition option: s16
  cout << "State Lane Change Right is updating......" << endl;
  fsm_state* next_state = NULL;
  if (isStopToKeepLane())
  {
    set_target_ego_velocity(2);
    set_target_ego_position(0);
    next_state = new KL(qV);
    cout << "next state: KL" << endl;
  }
  else
  {
    set_target_ego_velocity(1);
    set_target_ego_position(0);
    next_state = new Stop(qV);
    cout << "next state: Stop" << endl;
  }
  return next_state;
}

uint8_t Stop::turn2int()const{
    return 7;
}


bool Stop::isStopToKeepLane()
{
  bool condition = collision==false;
  return condition;
}
