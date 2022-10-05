#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include "boost/thread.hpp"

class MOVE_BASE {
    public:
        MOVE_BASE();
        void run();
        void loop();
        void goal_cb( geometry_msgs::Pose p);
        void interrupt_cb(std_msgs::Bool b);

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _goal_sub;
        ros::Subscriber _interrupt_sub;
        bool _new_goal;
        bool _interrupt;
        geometry_msgs::Pose _goal_pose;
};


MOVE_BASE::MOVE_BASE() {

    _interrupt_sub = _nh.subscribe("/interrupt_goal", 1, &MOVE_BASE::interrupt_cb, this);
    _goal_sub = _nh.subscribe( "/goal", 1, &MOVE_BASE::goal_cb, this);
    _new_goal = false;
    _interrupt = false;

}


void MOVE_BASE::goal_cb( geometry_msgs::Pose p ) {
    _goal_pose = p;
    _new_goal = true;
}

void MOVE_BASE::interrupt_cb( std_msgs::Bool b ) {
    _interrupt = b.data;
}

void MOVE_BASE::loop() {

    ros::Rate r(5);
    
    actionlib::SimpleActionClient< move_base_msgs::MoveBaseAction > ac("move_base", true);
    ROS_INFO("Waiting for move base action server");
    ac.waitForServer();
    ROS_INFO("Move base action server ready");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";

    
    while( ros::ok() ) {

        if( _new_goal ) {
            _new_goal = false;
            goal.target_pose.pose.position.x = _goal_pose.position.x;
            goal.target_pose.pose.position.y = _goal_pose.position.y;
            goal.target_pose.pose.orientation.w = 1.0;

            ac.sendGoal( goal );
    
        }


        actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action state: %s",state.toString().c_str());


        if( state.toString() == "ACTIVE" && _interrupt) {
            ac.cancelGoal();
            _interrupt = false;
        }

        r.sleep();
    }

}


void MOVE_BASE::run() {

    boost::thread loop_t( &MOVE_BASE::loop, this);
    ros::spin();

}

int main( int argc, char** argv ) {


    ros::init( argc, argv, "move_base_client" );
 
    MOVE_BASE mb;
    mb.run();

    return 0;
}