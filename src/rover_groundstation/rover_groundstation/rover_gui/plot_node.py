import rclpy
from rclpy.node import Node

#interfaces
from rover_interfaces.msg import MotorData
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

from collections import deque

import threading

class MultiPlot(Node):
    def __init__(self):
        super().__init__("plot_updater")

        self.create_subscription(MotorData, "/RoverTelem/roboclaw/enc_telem", self.motor_callback, 10)
        self.create_subscription(Imu, "/RoverTelem/bno055/imu", self.imu_callback, 10)
        self.create_subscription(PoseStamped, "/CubeRover_V1/pose", self.mocap_callback, 10)

        self.motor_data = {
            'time': deque(maxlen=100),
            'pos': [deque(maxlen=100) for _ in range(4)], #enc1, enc2, enc3, enc4
            'vel': [deque(maxlen=100) for _ in range(4)], # vel1, vel2, vel3, vel4
            'volt': [deque(maxlen=100) for _ in range(2)], #roboclaw volt1, volt2
            'amp': [deque(maxlen=100) for _ in range(4)], #roboclaw m1,m2,m3,m4 current draw
        }

        self.imu_data = {
            'time': deque(maxlen=100),
            'lin_accel': [deque(maxlen=100) for _ in range(3)], #x, y, z acceleration
        }

        self.mocap_data = {
            'time': deque(maxlen=100),
            'pose': [deque(maxlen=100) for _ in range(3)], #x, y, z pos
        }
        
        self.start = self.get_clock().now().nanoseconds * 1e-9

        self.motorlock = threading.Lock()
        self.imulock = threading.Lock()
        self.mocaplock = threading.Lock()

    def motor_callback(self, msg):

        with self.motorlock:
            self.motor_data['time'].append(self.get_clock().now().nanoseconds * 1e-9 - self.start)

            self.motor_data['pos'][0].append(msg.enc1)
            self.motor_data['pos'][1].append(msg.enc2)
            self.motor_data['pos'][2].append(msg.enc3)
            self.motor_data['pos'][3].append(msg.enc4)

            self.motor_data['vel'][0].append(msg.vel1)
            self.motor_data['vel'][1].append(msg.vel2)
            self.motor_data['vel'][2].append(msg.vel3)
            self.motor_data['vel'][3].append(msg.vel4)

            self.motor_data['amp'][0].append(msg.m1current)
            self.motor_data['amp'][1].append(msg.m2current)
            self.motor_data['amp'][2].append(msg.m3current)
            self.motor_data['amp'][3].append(msg.m4current)

            self.motor_data['volt'][0].append(14)
            self.motor_data['volt'][1].append(14)

    def imu_callback(self, msg):
        with self.imulock:
            self.imu_data['time'].append(self.get_clock().now().nanoseconds * 1e-9 - self.start)

            self.imu_data['lin_accel'][0].append(msg.linear_acceleration.x) #change to linaccel x
            self.imu_data['lin_accel'][1].append(msg.linear_acceleration.y) #change to linaccel y
            self.imu_data['lin_accel'][2].append(msg.linear_acceleration.z) #change to linaccel z

    def mocap_callback(self, msg):
        # with self.mocaplock:
        self.mocap_data['time'].append(self.get_clock().now().nanoseconds * 1e-9 - self.start)
        print(f"{msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z}")
        self.mocap_data.get('pose')[0].append(msg.pose.position.x) #change to linaccel x
        self.mocap_data.get('pose')[1].append(msg.pose.position.y) #change to linaccel y
        self.mocap_data.get('pose')[2].append(msg.pose.position.z) #change to linaccel z
            # print(f"{}, {msg.pose.position.y}, {msg.pose.position.z}")
