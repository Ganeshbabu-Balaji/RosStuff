#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from rospy import Time

distance = 0.5
stuck_timeout = rospy.Duration(1) 
side_distance_threshold = 1  

class StopAtWall(object):
    def __init__(self):
        rospy.init_node("walk_to_wall")
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin, angular=ang)
        self.last_movement_time = rospy.Time.now()

    def process_scan(self, data):

        if (data.ranges[0] == 0.0 or data.ranges[0] >= distance):
            self.twist.linear.x = 0.5
            self.twist.angular.z = 0  
            self.last_movement_time = rospy.Time.now()  
            rospy.loginfo("FORWARD")

        else:
            
            if(min(data.ranges[270:360]) < side_distance_threshold and min(data.ranges[0:90])) < side_distance_threshold:
                self.twist.linear.x = -1
                rospy.loginfo("Object on both sides")
                rospy.loginfo(min(data.ranges[270:360]))

            if min(data.ranges[270:360]) < side_distance_threshold:
                rospy.loginfo("Object detected on the right, turning left")
                rospy.loginfo(min(data.ranges[270:360]))
                self.twist.linear.x = 0.1
                self.twist.angular.z = -1 

            if min(data.ranges[0:90]) < side_distance_threshold:
                rospy.loginfo("Object detected on the left, turning right")
                rospy.loginfo(min(data.ranges[0:90]))
                self.twist.linear.x = 0.1
                self.twist.angular.z = 1 

        if rospy.Time.now() - self.last_movement_time > stuck_timeout:
            rospy.logwarn("Robot hit wall")

            if (data.ranges[180] == 0.0 or data.ranges[180] >= distance):
                rospy.loginfo("Object Front")
                rospy.loginfo(min(data.ranges[0:180]))
                self.twist.linear.x = -0.2
                self.twist.angular.z = 0
            else:
                rospy.loginfo("Object Behind")
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = StopAtWall()
    node.run()


