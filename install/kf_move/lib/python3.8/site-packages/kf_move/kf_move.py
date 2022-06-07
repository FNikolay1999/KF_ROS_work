import os
from re import X
import select
import sys
import rclpy

from geometry_msgs.msg import Twist #Скорость и угол поворота
from geometry_msgs.msg import Pose  #Позиция
from rclpy.qos import QoSProfile

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

#Для работы с фильтром Калмана
from datetime import datetime
import time #Замерить время, через которое отсылается cmd_vel
#from Kalman_filter_3rd_order import do_code2

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

class MinimalSubscriber(Node):

    def __init__(self):
        self.cur_point = (0.0,0.0) #pose робота, округленная до сотых

        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        point = msg.pose.pose.position #  the x,y,z
        self.cur_point = (round(point.x,2),round(point.y,2))
    
    def get_cur_point(self):
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        return self.cur_point

def main():

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    #Изменение скорости и угла
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('kf_move')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    
    minimal_subscriber = MinimalSubscriber()

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    control_linear_velocity = 0.0
    control_angular_velocity = 0.0

    try:
        while(1):
            
            """
            start_time = datetime.now()

            time.sleep(1)#задержка в течение 1 секунды
            
            print(datetime.now() - start_time)
            """

            #rclpy.spin(minimal_subscriber)
            #minimal_subscriber.destroy_node()
            print(minimal_subscriber.get_cur_point())

            #dt = 0.01

            
            twist = Twist()

            twist.linear.x = 0.2
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0

            pub.publish(twist)
            


    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
    
