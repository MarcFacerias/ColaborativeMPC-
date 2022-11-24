#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, TransformStamped, Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry 
import tf2_ros
import sys

class fake_localisation:

    def __init__(self):

        rospy.init_node('fake_localisation_gazebo', anonymous=True)
        self.pub = rospy.Publisher('puzzlebot/odom', Odometry, queue_size=10)
        rospy.Subscriber("gazebo/model_states", ModelStates, self.callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.o = Odometry()

    def callback(self,data):
        try:
            p = data.pose[data.name.index("puzzlebot")]
            
            self.o.pose.pose.position.x = p.position.x
            self.o.pose.pose.position.y = p.position.y
            self.o.pose.pose.position.z = p.position.z

            self.o.pose.pose.orientation.x = p.orientation.x
            self.o.pose.pose.orientation.y = p.orientation.y
            self.o.pose.pose.orientation.z = p.orientation.z
            self.o.pose.pose.orientation.w = p.orientation.w

            t = TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "puzzlebot/base_link"

            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z

            t.transform.rotation.x = p.orientation.x
            t.transform.rotation.y = p.orientation.y
            t.transform.rotation.z = p.orientation.z
            t.transform.rotation.w = p.orientation.w

            self.br.sendTransform(t)
        except:
            pass

    def run(self):
        rate = rospy.Rate(20) # 10hz
        
        while not rospy.is_shutdown():
            self.pub.publish(self.o)
            rate.sleep()



if __name__ == "__main__":
    aux = fake_localisation()
    aux.run()
