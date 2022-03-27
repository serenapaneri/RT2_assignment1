#! /usr/bin/env python


import rospy
import rt2_assignment1.msg
import actionlib
import actionlib.msg
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from rt2_assignment1.srv import Position
import math

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0

#publisher used for cmd_vel
pub_ = None

#action server
act_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

def clbk_odom(msg):
    """
    This is the callback function that is used to know 
    the actual position of the robot in the space.
    """
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]


def change_state(state):
    """
    With this function we are able to switch 
    between the different states of this service.
    """
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    """
    This function is used to normalized the angle
    """
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    """
    This function is used to adjust the yaw angle
    of the robot in a way that it is directly
    alligned with the target position to be reached.
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    """
    This function allows the robot to move
    straight to reach the goal position
    """
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # state change conditions
        #print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def fix_final_yaw(des_yaw):
    """
    This function allows the robot to adjust its
    goal orientation
    """
    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        #print ('Yaw error: [%s]' % err_yaw)
        change_state(3)
        
def done():
    """
    This function is used to stop the robot
    """
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
    
def go_to_point(goal):
    """
    This function is used to regulate the different
    states of the robot in a way to achieve the correct 
    behavior to be followed in order to reach the goal.
    """
    desired_position = Point()
    desired_position.x = goal.x
    desired_position.y = goal.y
    des_yaw = goal.theta
    
    rate = rospy.Rate(20)
    success = True
    
    change_state(0)
    
    feedback = rt2_assignment1.msg.TargetFeedback()
    result = rt2_assignment1.msg.TargetResult()
    
    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            done()
            break
        elif state_ == 0:
            feedback.stat = "Fixing the yaw"
            act_s.publish_feedback(feedback)
            fix_yaw(desired_position)
        elif state_ == 1:
            feedback.stat = "Angle aligned"
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position)
        elif state_ == 2:
            feedback.stat = "Target reached!"
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        result.ok= True
        act_s.set_succeeded(result)

def main():
    """
    This is the main function in which we create a publisher,
    a subscriber and an action server.
    """
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer('/go_to_point', rt2_assignment1.msg.TargetAction, go_to_point, auto_start=False)
    act_s.start()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
