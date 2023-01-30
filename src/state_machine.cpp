/**
 * \file state_machine.cpp
 * \brief This node implements the state machine
 * \author Serena Paneri 4506977
 * \version 1.0
 * \date 13/08/2022
 * 
 * \details
 * 
 * Services: <BR>
 *	°/user_interface
 *
 * Client: <BR>
 * 	°/position_server
 *
 * Action client: <BR>
 * 	°/go_to_point
 * 
 * Description:
 *
 * This node represents the state machine of the program and it only has two states
 * that are when the robot starts and when the robot stops. 
 * When the user starts the simulation by exploiting the user_interface, the state
 * machine calls the position_server in order to recieve the random x and y position 
 * and orientation theta, that will be the goal to be achieved by the robot in the 
 * simulation environemnt. Moreover it manages the action server go_to_point that
 * will drive the robot toward the goal position.
 * After achieving the goal position the state machine will make again the same 
 * requests as before until the user manually stops the behavior of the robot by
 * exploiting again the user_interface.
 */


#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/TargetAction.h"
#include "rt2_assignment1/RandomPosition.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "std_msgs/Int32.h"

bool start = false;

/**
 *
 * \param req: it is the request done by the user_interface
 * \param res: it is the response about the state of the robot
 *
 * \retval true
 *
 * This function contains the request made by the user in the user_interface to make 
 * the robot move.
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
 *
 * \retval 0
 *
 * This is the main function of the node state_machine and it menages a server, a  
 * client and an action client. Moverover it menages the two different states in which 
 * the robot could be.
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::Publisher target_pub = n.advertise<std_msgs::Int32>("robot_target", 1000);
   
   /* action server */
   actionlib::SimpleActionClient<rt2_assignment1::TargetAction> act_c("go_to_point", true);

   /* wait for the action server to come up */
   while(!act_c.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the action server to come up");
   }   
   
   std_msgs::Int32 trg;
   
   /* making a request to the position_server setting limits */
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		/* calling the position_server */
   		client_rp.call(rp);
   		rt2_assignment1::TargetGoal goal;
   		goal.x = rp.response.x;
   		goal.y = rp.response.y;
   		goal.theta = rp.response.theta;
   		
   		/* sending the x and y position and theta to the action server */
   		act_c.sendGoal(goal);
   		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
   		
   		act_c.waitForResult();
   		
   		/* checking if the goal has been reached or not */
               if(act_c.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                  trg.data = 1;
                  target_pub.publish(trg);
                  ROS_INFO("Hooray, target reached!");
               }
               else{
                  trg.data = 0;
                  target_pub.publish(trg);
                  ROS_INFO("The base failed to reach the target for some reason");
               }
   	}
   }
   return 0;
}
