#include <ros/ros.h>

#include <cross_fsm/query_variable.h>
#include <cross_fsm/condition.h>
#include <cross_fsm/trafficlight_condition.h>
#include <cross_fsm/stopsign_condition.h>
#include <cross_fsm/RvL_condition.h>
#include <cross_fsm/vehicle.h>
#include <cross_fsm/pub_class.h>


int main(int argv, char **argc){

    ros::init(argv, argc, "crossfsm_node");
    ros::Time::init();
    ros::Rate loop_rate(10);
    ros::NodeHandle n;

    pub pub2trajacy(n);

    queryVariable queryVar(n);
    queryVariable& queryV = queryVar;
    //state->action();
    Condition* cross_condition_ptr = nullptr;
    switch (queryV.getTrafficLight().light_id)
    {
    //according to msg in trafficlights to select the scenario
    case 1:
        cross_condition_ptr = new TrafficLightCondition(queryV);
        break;
    case 2:
        cross_condition_ptr = new TrafficLightCondition(queryV);
        break;
    case 3:
        cross_condition_ptr = new TrafficLightCondition(queryV);
        break;
    case 4:
        cross_condition_ptr = new TrafficLightCondition(queryV);
        break;
    case 0:
        cross_condition_ptr = new TrafficLightCondition(queryV);
        break;
    default://TODO RvL
        cross_condition_ptr = new RechtvorLinksCondition(queryV);
        break;
    }
    Condition& cross_condition = *cross_condition_ptr;

    //initialize the fsm
    fsm_state* state = new Buffer(cross_condition);

    while(ros::ok())
    {
        queryV.proccessData();
//        queryV.proccessDataSecuritycheck_left();
//        queryV.proccessDataSecuritycheck_right();
//        queryV.proccessDataSecuritycheck_forward();
        pub2trajacy.pubState(state, queryV);
        fsm_state* nextState = state->update_state();
        state = nextState;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;


}
