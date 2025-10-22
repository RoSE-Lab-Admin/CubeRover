import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rover_interfaces.action import TestCommand
from rover_interfaces.msg import RoverCommand

import numpy as np
import time

TRACKWIDTH = 31.5 / 2
ENCODER = 5281.7
RADIUS = 15
DELAY = 1200

class VelActionServer(Node):
    def __init__(self):
        super().__init__("vel_action_server")

        #create action server
        self.action_serv = ActionServer(
            self,
            TestCommand,
            'vel_action_server',
            execute_callback=self.execute_callback,
            #callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,

        )

        self.velRequest = RoverCommand()
        self.velRequest.type = 'V'
        self.velRequest.data = np.zeros(7, dtype=np.int32)

        #create publisher for motor command stream
        self.motorStream = self.create_publisher(RoverCommand, 'motor_stream', 20)

    def destroy_node(self):
        self.get_logger().info("bye!")
        super.destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info("Goal Request sent")
        v_L = goal_request.linear_speed
        v_R = goal_request.linear_speed
        if (goal_request.turning_radius < 1E6):
            if np.isclose(goal_request.turning_radius, 0):   # turn in place
                v_R = -v_L
            else:   #radius turn
                v_L = (goal_request.linear_speed * (goal_request.turning_radius - TRACKWIDTH)) / goal_request.turning_radius
                v_R = (goal_request.linear_speed * (goal_request.turning_radius + TRACKWIDTH)) / goal_request.turning_radius

        self.velRequest.data[0] = v_to_e(v_L)
        self.velRequest.data[1] = v_to_e(v_L)
        self.velRequest.data[2] = v_to_e(v_R)
        self.velRequest.data[3] = v_to_e(v_R)
        self.velRequest.data[4] = DELAY
        self.velRequest.data[5] = a_to_e((v_L / (goal_request.accel_deaccel_duration / 1000)))
        self.velRequest.data[6] = a_to_e((v_R / (goal_request.accel_deaccel_duration / 1000)))

        return GoalResponse.ACCEPT


    def cancel_callback(self, goal_handle):
        self.get_logger().info("recieved cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("executing action!")
        
        #rate = self.create_rate(1, self.get_clock())
        while not goal_handle.is_cancel_requested:
            self.get_logger().info("sent motor velocity")
            self.motorStream.publish(self.velRequest)
            #rclpy.spin_once(self, timeout_sec=0.75)
            time.sleep(1)
       
        result = TestCommand.Result()
        self.get_logger().info("Goal Cancelled!")
        goal_handle.canceled()
        self.stop_motors()
        self.motorStream.publish(self.velRequest)
        
        return result


    def stop_motors(self):
        self.velRequest.data[0] = 0
        self.velRequest.data[1] = 0
        self.velRequest.data[2] = 0
        self.velRequest.data[3] = 0
        self.velRequest.data[4] = DELAY
        self.velRequest.data[5] = 500
        self.velRequest.data[6] = 500

        


def v_to_e(v):
    return int((v/RADIUS)/(2*np.pi)*ENCODER)

def a_to_e(a):
    return int((a/RADIUS)/(2*np.pi)*ENCODER)


def main(args=None):
    rclpy.init(args=args)
    node = VelActionServer()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()