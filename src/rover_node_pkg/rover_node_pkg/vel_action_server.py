import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rover_interfaces.action import TestCommand
from rover_interfaces.msg import RoverCommand
import numpy as np
import time

TRACKWIDTH = 40.96 / 2
ENCODER = 5281.7
RADIUS = 15

class VelActionServer(Node):
    def __init__(self):
        super().__init__("velocity_action_server")
        self.action_serv = ActionServer(
            self,
            TestCommand,
            'TestingActionServer',
            self.action_callback
        )
        self.motorStream = self.create_publisher(RoverCommand, 'motor_stream', 20)

    def action_callback(self, goal_handle):
        #create velocity service command
        velRequest = RoverCommand()
        velRequest.type = 'V'
        velRequest.data = np.zeros(7, dtype=np.int32)
        v_L = goal_handle.request.linear_speed
        v_R = goal_handle.request.linear_speed
        if (goal_handle.request.turning_radius < 1E6):
            if (goal_handle.request.turning_radius == 0):   # turn in place
                v_R = -v_L
            else:   #radius turn
                v_L = goal_handle.request.linear_speed * (goal_handle.request.turning_radius - TRACKWIDTH) / goal_handle.request.turning_radius
                v_R = goal_handle.request.linear_speed * (goal_handle.request.turning_radius + TRACKWIDTH) / goal_handle.request.turning_radius

        start = time.time()
        driveTime = goal_handle.request.run_duration - goal_handle.request.accel_deacel_duration
        
        velRequest.data[0] = v_to_e(v_L)
        velRequest.data[1] = v_to_e(v_L)
        velRequest.data[2] = v_to_e(v_L)
        velRequest.data[3] = v_to_e(v_L)
        velRequest.data[4] = 250
        velRequest.data[5] = a_to_e((goal_handle.request.linear_speed / (goal_handle.request.accel_deacel_duration / 1000)))
        velRequest.data[6] = a_to_e((goal_handle.request.linear_speed / (goal_handle.request.accel_deacel_duration / 1000)))
        
        
        while ((time.time() - start) < (driveTime / 1000)):
            self.motorStream.publish(velRequest)
            self.get_logger.info(f"time: {start - time.time()}")
            time.sleep(0.05)
        
        
        velRequest.data[0] = v_to_e(0)
        velRequest.data[1] = v_to_e(0)
        velRequest.data[2] = v_to_e(0)
        velRequest.data[3] = v_to_e(0)
        velRequest.data[4] = 250
        velRequest.data[5] = a_to_e((goal_handle.request.linear_speed / (goal_handle.request.accel_deacel_duration / 1000)))
        velRequest.data[6] = a_to_e((goal_handle.request.linear_speed / (goal_handle.request.accel_deacel_duration / 1000)))

        while ((time.time() - start) < (goal_handle.request.run_duration / 1000)):
            self.motorStream.publish(velRequest)
            self.get_logger.info(f"time: {start - time.time()}")
            time.sleep(0.05)

        goal_handle.succeed()
        return 
        


def v_to_e(v):
    return int((v/RADIUS)/(2*np.pi)*ENCODER)

def a_to_e(a):
    return int((a/RADIUS)/(2*np.pi)*ENCODER)


def main(args=None):
    rclpy.init(args=args)
    node = VelActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()