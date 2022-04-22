#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
	
   class PositionService : public rclcpp::Node
   {
      public:
      
         /**
          * This is the initialization of the service for RandomPosition.
          */
   
         PositionService(const rclcpp::NodeOptions & options) : Node("position_service", options)
         {
            service = this->create_service<RandomPosition>("/position_server", std::bind(&PositionService::myrandom, this, _1,_2,_3));
         }
         
      private:
      
         /**
          * This function simply returns a random number within the
          * interval [N,M].
          */ 
   
         double randMToN(double M, double N)
         {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }
         
         /**
          * This function is the callback used by the server in order to
          * generate a random x and y position plus an orientation theta
          * in the space.
          */ 

         bool myrandom(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<RandomPosition::Request> request, const std::shared_ptr<RandomPosition::Response> response)
         {
            (void)request_header;
            response->x = randMToN(request->x_min, request->x_max);
            response->y = randMToN(request->y_min, request->y_max);
            response->theta = randMToN(-3.14, 3.14);
            return true;
         }

         rclcpp::Service<RandomPosition>::SharedPtr service;
      
   };
   
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::PositionService) 
