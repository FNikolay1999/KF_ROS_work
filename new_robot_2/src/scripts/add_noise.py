#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
#
# Modified by AutomaticAddison.com

from __future__ import print_function
import time # Time library
from copy import deepcopy # Modifying a deep copy does not impact the original

from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_navigator import BasicNavigator, NavigationResult # Helper module

from rclpy.node import Node
from nav_msgs.msg import Odometry
#Для работы с фильтром Калмана
from datetime import datetime, timedelta


class SubWheelOdom(Node):

    def __init__(self):
        self.cur_point = (0.0,0.0) #pose робота, округленная до сотых

        #self.dt = 1.0
        self.dt = 0.1
        self.cur_dt = datetime.now() #Текущая разница во времени
        self.start_time = datetime.now() #Отчёт, который будет работать с разницей

        self.work_start_time = datetime.now()
        self.cur_t = 0.0
        self.cur_t_s = 0.0

        super().__init__('sub_wheel_odom')
        self.subscription = self.create_subscription(
            Odometry,
            '/wheel/odometry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        self.cur_dt = (datetime.now() - self.start_time).total_seconds()

        if(self.cur_dt > self.dt):
            point = msg.pose.pose.position #  the x,y,z
            self.cur_point = (round(point.x,2),round(point.y,2))

            self.cur_t = datetime.now().microsecond / 1000
            self.cur_t_s = datetime.now().second

            print('%-30s %2d %8.3f %8s' % ('wheel_odom -> time (second, microsecond):', self.cur_t_s, self.cur_t, " pose: "), end=' ')
            print(self.cur_point)
            #print("odom -> time (microsecond):" , self.cur_t, " pose: " ,self.cur_point, sep=' ')

            self.start_time = datetime.now()

    def get_cur_point(self):
        return self.cur_point

"""
class PubWorkOdom(Node):

    def __init__(self):
        super().__init__('pub_work_odom')
        self.publisher_ = self.create_publisher(Odometry, '/pub_work_odom', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
"""


'''
Pick up an item in one location and deliver it to another. 
The assumption is that there is a person at the pick and delivery location
to load and unload the item from the robot.
'''
def main():

  # Start the ROS 2 Python Client Library
  rclpy.init()

  # Для получения позиции
  global sub_wheel_odom
  sub_wheel_odom = SubWheelOdom()

  #global pub_work_odom
  #pub_work_odom

  rclpy.spin(sub_wheel_odom)

  sub_wheel_odom.destroy_node()
  exit(0)

if __name__ == '__main__':
  main()
