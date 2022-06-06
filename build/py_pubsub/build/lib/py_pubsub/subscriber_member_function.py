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

from ast import Constant
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Vector3
from std_msgs.msg import String

INF=1000.0

class LidarSubscriber(Node):
    min_dist=INF
    def __init__(self,min_dist):
        super().__init__('ariadna_collision_avoidance')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        meas_min_dist = INF
        for range in msg.ranges:
            if range < meas_min_dist:
                meas_min_dist = range
            if meas_min_dist < min_dist:
                min_dist=meas_min_dist
                

        self.get_logger().info('I heard: "%.5f"' % meas_min_dist)


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('ariadna_collision_avoidance')
        self.publisher_ = self.create_publisher(String, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'linear:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n angular:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n ---\n' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


class CollisionAvoidance():

    trig_dist=1.0
    def colav(self, new_min_dist, args=None):
        self.min_dist=new_min_dist
        rclpy.init(args=args)

        lidar_subscriber = LidarSubscriber(INF)
        lidar_publisher = LidarPublisher()


        while lidar_subscriber.min_dist < self.trig_dist:
            rclpy.spin(lidar_subscriber)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        lidar_subscriber.destroy_node()

        rclpy.spin(lidar_publisher)
        lidar_publisher.destroy_node()
        rclpy.shutdown()




        

        
        rclpy.shutdown()


def main(args=None):
    colav=CollisionAvoidance()
    colav.colav(1.0)


if __name__ == '__main__':
    main()

# To wysyłac na /cmd_vel
# linear:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# angular:
#   x: 0.0
#   y: 0.0
#   z: 0.0
# ---
