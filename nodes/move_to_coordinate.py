#!/usr/bin/env python3
#modified from Gilbert's code 
#reference: https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_example/nodes/turtlebot3_pointop_key

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time

agv_map="""
Control your AGV
Select 2 location
+----+----+----+----+----+
| 1  | 2  | 3  | 4  | 5  |
+----+----+----+----+----+
| 6  | 7  | 8  | 9  | 10 |
+----+----+----+----+----+
| 11 | 12 | 13 | 14 | 15 |
+----+----+----+----+----+
| 16 | 17 | 18 | 19 | 20 |
+----+----+----+----+----+
| 21 | 22 | 23 | 24 | 25 |
+----+----+----+----+----+
type non number character to shutdown
"""


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (self.position, rotation) = self.get_odom()
        self.last_rotation = 0
        self.linear_speed = 1
        self.angular_speed = 1

        print("Select the 1st point")
        (goal_x1, goal_y1, map_label1) = self.get_map_coordinate()
        print("Select the 2nd point")
        (goal_x2, goal_y2, map_label2) = self.get_map_coordinate()
        
        #move robot to sepcify coordinate
        rospy.loginfo("Moving to first location point " + str(map_label1))
        self.move_to_point(self.position.x, goal_y1)
        self.move_to_point(goal_x1, goal_y1)
        #self.rotate_to_point(goal_z1)
        rospy.loginfo("Frist location reached!")
        rospy.loginfo("Stop for 5 seconds...")
        time.sleep(5)
        rospy.loginfo("Moving to second location point " + str(map_label2))
        self.move_to_point(self.position.x, goal_y2)
        self.move_to_point(goal_x2, goal_y2)
        #self.rotate_to_point(goal_z2)
        rospy.loginfo("Second loaction reached!")
        rospy.loginfo("Stop for 5 seconds...")
        time.sleep(5)
        rospy.loginfo('Moving back to charger')
        self.move_to_point(self.position.x, 0.05)
        self.move_to_point(0, 0)
        self.rotate_to_point(0)
    
        
    def move_to_point(self, goal_x, goal_y):
        goal_distance = sqrt(pow(goal_x - self.position.x, 2) + pow(goal_y - self.position.y, 2))
        distance = goal_distance

        while distance > 0.05:
            (self.position, rotation) = self.get_odom()
            x_start = self.position.x
            y_start = self.position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if self.last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif self.last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            self.move_cmd.angular.z = self.angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            self.move_cmd.linear.x = min(self.linear_speed * distance, 0.1)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            self.last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        self.cmd_vel.publish(Twist())
    
    def rotate_to_point (self, goal_z):
        (self.position, rotation) = self.get_odom()
        while abs(rotation - goal_z) > 0.05:
            (self.position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = -0.5
                else:
                    self.move_cmd.linear.x = 0.00
                    self.move_cmd.angular.z = 0.5
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_map_coordinate(self):
        map_label = input("Type the location number: ")
        if int(map_label) == 1:
            return -4.5, 4.5, 1
        elif int(map_label) == 2:
            return -3.5, 4.5, 2
        elif int(map_label) == 3:
            return -2.5, 4.5, 3
        elif int(map_label) == 4:
            return -1.5, 4.5, 4
        elif int(map_label) == 5:
            return -0.5, 4.5, 5

        elif int(map_label) == 6:
           return -4.5, 3.5, 6
        elif int(map_label) == 7:
            return -3.5, 3.5, 7
        elif int(map_label) == 8:
           return -2.5, 3.5, 8
        elif int(map_label) == 9:
            return -1.5, 3.5, 9
        elif int(map_label) == 10:
            return -0.5, 3.5, 10

        elif int(map_label) == 11:
             return -4.5, 2.5, 11
        elif int(map_label) == 12:
            return -3.5, 2.5, 12
        elif int(map_label) == 13:
            return -2.5, 2.5, 13
        elif int(map_label) == 14:
            return -1.5, 2.5, 14
        elif int(map_label) == 15:
            return -0.5, 2.5, 15

        elif int(map_label) == 16:
            return -4.5, 1.5, 16
        elif int(map_label) == 17:
            return -3.5, 1.5, 17
        elif int(map_label) == 18:
            return -2.5, 1.5, 18
        elif int(map_label) == 19:
            return -1.5, 1.5, 19
        elif int(map_label) == 20:
            return -0.5, 1.5, 20
            
        elif int(map_label) == 21:
            return -4.5, 0.5, 21
        elif int(map_label) == 22:
            return -3.5, 0.5, 22
        elif int(map_label) == 23:
            return -2.5, 0.5, 23
        elif int(map_label) == 24:
            return -1.5, 0.5, 24
        elif int(map_label) == 25:
            return -0.5, 0.5, 25

        else:
            rospy.loginfo("Invalid input")
            self.shutdown
        
    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            print(agv_map)
            GotoPoint()
    except:
        rospy.loginfo("shutdown program.")
  