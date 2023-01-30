/**
 * \file position_service.cpp
 * \brief This node generates the x,y position and the orientation theta
 * \author Serena Paneri 4506977
 * \version 1.0
 * \date 13/08/2022
 * 
 * \details
 * 
 * Services: <BR>
 *	°/position_server
 * 
 * Description:
 
 * This node implements a service and its goal is to generate the x and y
 * position plus the orientation theta. These are random numbers generated
 * within a given range, and they represent the target position that 
 * the robot is going to achive.
 */


#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 *
 * \param M: it is the lower bound of the range
 * \param N: it is the upper bound of the range
 *
 * \retval randMTon: it returns a random value within the range 
 *
 * This function returns a random number within the interval [M,N].
 */ 
 
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 *
 * \param req: it is the request done by client of a target position and orientation
 * \param res: it is the response of the server providing the target position and orientation
 * 
 * \retval True
 *
 * This function generates the random target position and orientation to be achieved by
 * the robot.
 */
 
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


/**
 * This is the main function of the position_service node.
 */
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
