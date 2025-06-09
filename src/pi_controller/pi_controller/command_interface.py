import rclpy
from rclpy.node import Node
from rover_interfaces.srv import VelocityCommand, PositionCommand, TurnCommand, RoverCommand
import time
import numpy as np

class CommandNode(Node):
    def __init__(self):
        super().__init__("Command_Interface")
        self.get_logger().info("CommandNode Init")

        self.serialsrv = self.create_client(RoverCommand, 'SerialCommand')

        while not self.serialsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Connected to serial service")


        self.create_service(VelocityCommand, 'VelocityCommand', self.vel_callback)
        # self.create_service(PositionCommand, 'VelocityCommand', self.pos_callback)
        # self.create_service(TurnCommand, 'VelocityCommand', self.turn_callback)
    

    def vel_callback(self, request, response):
        
        serial_request = RoverCommand.Request()
        serial_request.type = 'V'
        serial_request.data = np.zeros(7, dtype=np.int32)
        serial_request.data[0] = request.l1
        serial_request.data[1] = request.l2
        serial_request.data[2] = request.r1
        serial_request.data[3] = request.r2
        serial_request.data[4] = request.timetodrive
        serial_request.data[5] = request.accel
        serial_request.data[6] = request.deaccel

        self.get_logger().info(f"sending packet: {request.l1}, {request.l2}, {request.r1}, {request.r2}, {request.timetodrive}, {request.accel}")

        future = self.serialsrv.call_async(serial_request)

        if future.result() is not None:
            response.success = future.result().success
        else:
            response.success = False

        return response
    
    def destroy_node(self):
        return
    

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



