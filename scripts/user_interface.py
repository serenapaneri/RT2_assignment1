import rospy
import time
from rt2_assignment1.srv import Command
import actionlib
import actionlib.msg
import rt2_assignment1.msg

def main():
    """
    This is the main function of the user_interface node. 
    It asks the user to start or to stop the robot, by pressing 
    1 and 0 respectively, and it calls the service implemented 
    in the finite_state_machine node. An action client has been
    implemented in order to stop the robot when requested.
    """
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    
    #action client 
    act_c = actionlib.SimpleActionClient("go_to_point",rt2_assignment1.msg.TargetAction)
    act_c.wait_for_server()
    
    time.sleep(10)
    rate = rospy.Rate(20)
    
    x = int(input("\nPress 1 to start the robot "))
    
    while not rospy.is_shutdown():
        #if the user presses 1
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress whatever number (except 1) to stop the robot "))
        #if the user presses another number, he send the command to stop the robot
        else:
            print("The robot has been stopped")
            act_c.cancel_all_goals()
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
