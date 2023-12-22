#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def odom_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Linear Velocity X: %f", data.twist.twist.linear.x)
    rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Y: %f", data.twist.twist.linear.y)
    rospy.loginfo(rospy.get_caller_id() + "Linear Velocity Z: %f", data.twist.twist.linear.z)
    rospy.loginfo(rospy.get_caller_id() + "Angular Velocity X: %f", data.twist.twist.angular.x)
    rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Y: %f", data.twist.twist.angular.y)
    rospy.loginfo(rospy.get_caller_id() + "Angular Velocity Z: %f", data.twist.twist.angular.z)
    rospy.loginfo(rospy.get_caller_id() + "Position X: %f", data.pose.pose.position.x)
    rospy.loginfo(rospy.get_caller_id() + "Position Y: %f", data.pose.pose.position.y)
    rospy.loginfo(rospy.get_caller_id() + "Position Z: %f", data.pose.pose.position.z)
    
def odom_reader():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cart_odom_subscriber', anonymous=True)

    rospy.Subscriber("cart_diff_drive_controller/odom", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    
def drive_controller():
    pub = rospy.Publisher('cart_diff_drive_controller/cmd_vel', Twist, queue_size=10)
    #rospy.init_node('cart_drive_publisher', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    control = Twist();
    control.linear.x = 0.05;
    control.angular.z = 0.15;
    while not rospy.is_shutdown():
        pub.publish(control)
        rate.sleep()
        control.linear.x += 0.05;
        control.angular.z += 0.15;

if __name__ == '__main__':
    odom_reader()
    drive_controller()

