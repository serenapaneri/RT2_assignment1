#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * This function simply return a random number 
 * between the interval [N,M].
 */ 
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }


/**
 * This function allows to generate the random 
 * target position and orientation to be achieved
 * by the robot.
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}


/**
 * This is the main function of the service and 
 * inside that we initialize the server.
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
