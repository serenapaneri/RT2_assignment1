#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/TargetAction.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;

/**
 * This function contained the request made by the user
 * in the user_interface to make the robot move and to
 * make the robot stop.
 */

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

/**
 * This is the main function of the node state_machine and it menages
 * a server, a client and an action client. Moverover it menages
 * the two different states in which the robot could be.
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   
   //action server
   actionlib::SimpleActionClient<rt2_assignment1::TargetAction> act_c("go_to_point", true);

   //wait for the action server to come up
   while(!act_c.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the action server to come up");
   }   
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
   		rt2_assignment1::TargetGoal goal;
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		
   		act_c.sendGoal(goal);
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
   		
   		act_c.waitForResult();
   		
               if(act_c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                  ROS_INFO("Hooray, target reached!");
               else
                  ROS_INFO("The base failed to reach the target for some reason");
   	}
   }
   return 0;
}
