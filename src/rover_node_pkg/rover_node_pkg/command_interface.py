import rclpy
from rclpy.node import Node
from rover_interfaces.srv import VelocityCommand, PositionCommand, TurnCommand, RoverCommand
import time
import numpy as np

ENCODER = 5281.7
RADIUS = 15

class CommandNode(Node):
    def __init__(self):
        super().__init__("command_interface")
        self.get_logger().info("CommandNode Init")

        self.serialsrv = self.create_client(RoverCommand, 'SerialCommand')

        while not self.serialsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Connected to serial service")


        self.create_service(VelocityCommand, 'VelocityCommand', self.vel_callback)
        # self.create_service(PositionCommand, 'VelocityCommand', self.pos_callback)
        # self.create_service(TurnCommand, 'VelocityCommand', self.turn_callback)
    

    def vel_callback(self, request, response): #TODO -> adjust values to be entered in as cm/s not encoder values
        
        serial_request = RoverCommand.Request()
        serial_request.type = 'V'
        serial_request.data = np.zeros(7, dtype=np.int32)
        serial_request.data[0] = v_to_e(request.l1)
        serial_request.data[1] = v_to_e(request.l2)
        serial_request.data[2] = v_to_e(request.r1)
        serial_request.data[3] = v_to_e(request.r2)
        serial_request.data[4] = request.timetodrive
        serial_request.data[5] = a_to_e(request.accel)
        serial_request.data[6] = a_to_e(request.deaccel)

        self.get_logger().info(f"sending packet: {request.l1}, {request.l2}, {request.r1}, {request.r2}, {request.timetodrive}, {request.accel}")

        future = self.serialsrv.call_async(serial_request)

        if future.result() is not None:
            response.success = future.result().success
        else:
            response.success = False

        return response
    
    def destroy_node(self):
        return super().destroy_node()
    

def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()



def v_to_e(v):
    return (v/RADIUS)/(2*np.pi)*ENCODER

def a_to_e(a):
    return (a/RADIUS)/(2*np.pi)*ENCODER