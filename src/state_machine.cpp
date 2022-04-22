#include <cinttypes>
#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{

   class StateMachine : public rclcpp::Node
   {
      public:
      
         /**
          * This includes the initialization of the service for the Command,
          * which is sent by the user, and two clients, one of them for 
          * RandomPosition that get the random position to reach by the robot,
          * and the other one for Positio that allow the robot to move 
          * directly throungh the target position.
          */ 
   
         StateMachine(const rclcpp::NodeOptions & options) : Node("state_machine", options)
         {
            service = this->create_service<Command>("/user_interface", std::bind(&StateMachine::user_interface, this, _1,_2,_3));
          
            client_rp = this->create_client<RandomPosition>("/position_server");
            while (!client_rp->wait_for_service(std::chrono::seconds(1)))
            {
               if (!rclcpp::ok()) 
               {
                  RCLCPP_ERROR(this->get_logger(), "client random position interrupted while waiting for service to appear.");
                  return;
               }
               RCLCPP_INFO(this->get_logger(), "waiting for service random position to appear...");
            }
          
            client_p = this->create_client<Position>("/go_to_point");
            while (!client_p->wait_for_service(std::chrono::seconds(1)))
            {
               if (!rclcpp::ok()) 
               {
                  RCLCPP_ERROR(this->get_logger(), "client position interrupted while waiting for service to appear.");
                  return;
               }
               RCLCPP_INFO(this->get_logger(), "waiting for service position to appear...");
            } 
         }
       
         /**
          * This function is used to menage a request made by RandomPosition
          * that asks for a random position and orientation in the space
          * in order to communicate after the response to the go_to_point
          * node that will guide the robot through the goal.
          */ 
       
         void handle_states()
         {
             auto rp = std::make_shared<RandomPosition::Request>();
             rp->x_max= 5.0;
             rp->x_min= -5.0;
             rp->y_max= 5.0;
             rp->y_min= -5.0;
             p = std::make_shared<Position::Request>();
          
             if(this->start)
             {
                using ServiceResponseFuture = rclcpp::Client<RandomPosition>::SharedFuture;
                auto response_received_callback = [this](ServiceResponseFuture future) 
                {
                   p->x = future.get()->x;
                   p->y = future.get()->y;
                   p->theta = future.get()->theta; 
                   std::cout << "\nGoing to the position: x= " << p->x << " y= " <<p->y << " theta = " <<p->theta << std::endl;
                 
                   using ServiceResponseFuture = rclcpp::Client<Position>::SharedFuture;
                   auto response_position = [this](ServiceResponseFuture future_)
                   {
                      if(future_.get()->ok)
                      {
                         std::cout << "Position reached" << std::endl;
                         handle_states();
                      }  
                   };
                   auto future_result_ = client_p->async_send_request(p, response_position);
                 
                };   
                auto future_result = client_rp->async_send_request(rp, response_received_callback);
            }
         }  
   
      private:
      
         bool start = false;
         
         /**
          * This function contained the request made by the user
          * in the user_interface to make the robot move and to
          * make the robot stop.
          */
      
         bool user_interface(const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<Command::Request> request, const std::shared_ptr<Command::Response> response)
         {
            (void)request_header;
            if (request->command == "start")
            {
    	       this->start = true;
    	       this->handle_states();
            }
            else {
    	       this->start = false;
            }
            return true;
         }
      
         rclcpp::Service<Command>::SharedPtr service;
         rclcpp::Client<Position>::SharedPtr client_p;
         rclcpp::Client<RandomPosition>::SharedPtr client_rp;
         std::shared_ptr<Position::Request> p;

   };

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)
