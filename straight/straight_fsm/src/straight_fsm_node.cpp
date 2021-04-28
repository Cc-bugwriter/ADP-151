#include <ros/ros.h>
#include <straight_fsm/vehicle.h>
#include <straight_fsm/query_variable.h>
#include <straight_fsm/pub_class.h>


int main(int argv, char **argc){

    ros::init(argv, argc, "straightfsm_node");
    ros::Time::init();
    ros::Rate loop_rate(10);
    ros::NodeHandle n;

    pub pub2trajectory(n);
    queryVariable qV;


    fsm_state* state = new Buffer(qV);
    static const clock_t startTime_w = ros::Time::now().toNSec();

    while(ros::ok()){

        qV.get_otherObjects();

        state->action();

        fsm_state* nextState = state->update_state(qV);
        pub2trajectory.pubState(state);

        state = nextState;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    }
