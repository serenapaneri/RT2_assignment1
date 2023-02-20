import rospy
import time
from rt2_assignment1.srv import Command
from rt2_assignment1.srv import Finish

finish = False

def main():
    """
    This is the main function of the user_interface node. 
    It asks the user to start or to stop the robot, by pressing 
    1 and 0 respectively, and it calls the service implemented 
    in the finite_state_machine node.
    """
    global finish
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    finish_client = rospy.ServiceProxy('/finished', Finish)
    time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            ui_client("start")
            x = int(input("\nPress 0 to stop the robot "))
        else:
            # print("Please wait, the robot is going to stop when the position will be reached")
            res = finish_client()
            finish = res.finish

            if finish == False:
                time.sleep(0.1)
            elif finish == True:
                time.sleep(1)
                ui_client("stop")
                x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
