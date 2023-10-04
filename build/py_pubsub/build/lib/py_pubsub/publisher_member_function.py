# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
topic = "turtle1/cmd_vel"


import math


DEG2RAD = math.pi / 180.0
RAD2DEG = 180 / math.pi

CENTER = 0
LEFT = 1
RIGHT = 2

LINEAR_VELOCITY = 5
ANGULAR_VELOCITY = 1.5

GET_TB3_DIRECTION = 0
TB3_DRIVE_FORWARD = 1
TB3_RIGHT_TURN = 2
TB3_LEFT_TURN = 3




from rclpy.qos import QoSProfile, QoSHistoryPolicy




import math
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        # Weirdness with the qos argument; it accepts either a QoSProfile or
        # an int, in the case of an int this seems to be the depth argument,
        # so passing an int x essentially creates a QosProfile with HistoryPolicy.KeepLast
        # and depth of x. So LaserScan differs from the others only in that it has a greater depth.
        self.cmd_vel_pub_ = self.create_publisher(Twist, topic, qos) #TODO: qos
        self.scan_sub_ = self.create_subscription(LaserScan, "scan", self.scan_callback,
                                                 10)
        self.odom_sub_ = self.create_subscription(Odometry, "odom", self.odom_callback, qos)
        self.timer = self.create_timer(0.01, self.update_callback)
        self.scan_data_ = [0.0, 0.0, 0.0]
        self.robot_pose_ = 0.0
        self.prev_robot_pose_ = 0.0


    def update_cmd_vel(self, lin, ang):
        twist = Twist()
        twist.cmd_vel.linear.x = lin
        twist.cmd_vel.angular.z = ang
        self.cmd_vel_pub_.publish(twist.cmd_vel)


    


    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.robot_pose_ = yaw


    def scan_callback(self, msg):
        scan_angle = [0, 30, 330]
        for num in range(3):
            if math.isinf(msg.ranges[num]):
                self.scan_data_[num] = msg.range_max # If inf, set to max range
            else:
                self.scan_data_[num] = msg.ranges[scan_angle[num]] # Otherwise, fetch the distance
                                                                   # at the angle given (0, 30, 330)

    def update_callback(self):
        turtlebot3_state_num = 0
        escape_range = 30.0 * DEG2RAD
        check_forward_dist = 0.7
        check_side_dist = 0.6

        if turtlebot3_state_num == GET_TB3_DIRECTION:
            if self.scan_data_[CENTER] > check_forward_dist:
                if self.scan_data_[LEFT] < check_side_dist:
                    self.prev_robot_pose_ = self.robot_pose_
                    turtlebot3_state_num = TB3_RIGHT_TURN
                elif self.scan_data_[RIGHT] < check_side_dist:
                    self.prev_robot_pose_ = self.robot_pose_
                    turtlebot3_state_num = TB3_LEFT_TURN
                else:
                    turtlebot3_state_num = TB3_DRIVE_FORWARD
            if self.scan_data_[CENTER] < check_forward_dist:
                self.prev_robot_pose_ = self.robot_pose_
                turtlebot3_state_num = TB3_RIGHT_TURN
        elif turtlebot3_state_num == TB3_DRIVE_FORWARD:
            self.update_cmd_vel(LINEAR_VELOCITY, 0.0)
        elif turtlebot3_state_num == TB3_RIGHT_TURN:
            if abs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                turtlebot3_state_num = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, -1 * ANGULAR_VELOCITY) # Turn right without going forward
        elif turtlebot3_state_num == TB3_LEFT_TURN:
            if abs(self.prev_robot_pose_ - self.robot_pose_) >= escape_range:
                turtlebot3_state_num = GET_TB3_DIRECTION
            else:
                self.update_cmd_vel(0.0, ANGULAR_VELOCITY) # Turn left without going forward
        else:
            turtlebot3_state_num = GET_TB3_DIRECTION

        print(f"Pose: {self.robot_pose_}")
        print(f"Scan data: {self.scan_data_}")










def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
