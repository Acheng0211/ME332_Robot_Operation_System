#!/usr/bin/env python  
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64

def callback(data):
    print (data.bounding_boxes[0].Class)

    if data.bounding_boxes[0].Class == 'vase':
        # pub1 = rospy.Publisher('/arm/joint1_position_controller/command', Float64, queue_size=10)
        # pub1.publish(1.0)
        # print("arm joint1 1.0")
        pub2 = rospy.Publisher('/arm/joint2_position_controller/command', Float64, queue_size=10)
        pub2.publish(1.0)
        print("arm joint2 1.0")
        pub3 = rospy.Publisher('/arm/joint3_position_controller/command', Float64, queue_size=10)
        pub3.publish(1.0)
        print("arm joint3 1.0")
        pub4 = rospy.Publisher('/arm/joint4_position_controller/command', Float64, queue_size=10)
        pub4.publish(1.0)
        print("arm joint4 1.0")
        pub5 = rospy.Publisher('/arm/joint5_position_controller/command', Float64, queue_size=10)
        pub5.publish(1.0)
        print("arm joint5 1.0")
        pub6 = rospy.Publisher('/arm/joint6_position_controller/command', Float64, queue_size=10)
        pub6.publish(1.0)
        print("arm joint6 1.0")
        # pub8 = rospy.Publisher('/arm/finger_joint1_gripper_controller/command', Float64, queue_size=10)
        # pub8.publish(-1.0)
        # print("gripper joint1 -1.0")


def listener():
    rospy.init_node('topic_subscriber')

    sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
