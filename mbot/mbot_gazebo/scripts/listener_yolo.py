#!/usr/bin/env python  
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist


def callback(data):
    print (data.bounding_boxes[0].Class)
    if data.bounding_boxes[0].Class == 'person':
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.4
        desired_velocity.angular.z = 0
        pub.publish(desired_velocity)
        print("go on")
        print(desired_velocity.linear.x)
    else:
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        desired_velocity = Twist()
        desired_velocity.linear.x = 0
        desired_velocity.angular.z = 0
        pub.publish(desired_velocity)
        print("stop")
   
   

def listener():
    rospy.init_node('topic_subscriber')

    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
