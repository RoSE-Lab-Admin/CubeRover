import rclpy
from rclpy.node import Node
from rover_interfaces.srv import VelocityCommand
import time
from controller_mode.controller_input import ControllerReader
import controller_mode.input_converter as ic

class ControllerNode(Node):
    def __init__(self):
        super().__init__("ControllerNode")
        self.get_logger().info("Started Controller")

        time.sleep(1) #wait for 1 second

        self.velsrv = self.create_client(VelocityCommand,'VelocityCommand') #establish service
        while not self.velsrv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Command Interface not available, waiting again...')
        self.get_logger().info('Connected to Command Interface')

        self.controller = ControllerReader()
        self.controller.connect()

        self.create_timer(0.1, self.send_input)

    def send_input(self):
        data = self.controller.get_input()
        # print("Controller Data:", data)
        velRequest = VelocityCommand.Request()
        DELAY = 300
        velRequest.accel = 1500
        velRequest.deaccel = 1500

        if data is not None:
            rX = data[0]
            rY = data[1]
            lT = data[2] + 1 #changes values from -1-1 to 0-2
            rT = data[3] + 1

            #drift reduction
            if rX < .1 and rX > -.1:
                rX = 0

            if rY < .1 and rY > -.1:
                rY = 0
            print(f"rX: {rX}, rY: {rY}, lT: {lT}, rT: {rT}")
            #if nothing is being pressed, base case:
            if lT == 0 and rT == 0 and rX == 0 and rY == 0:
                velRequest.l1 = 0
                velRequest.l2 = 0
                velRequest.r1 = 0
                velRequest.r2 = 0
                velRequest.timetodrive = DELAY


            #if turning
            elif (rT or lT) and (rX or rY):
                if lT != 0:
                    trig = -1*lT
                else:
                    trig = rT
                angle, radius, vel1, vel2 = ic.turn_calc(rX, rY, trig)
                velRequest.l1 = int(vel1)
                velRequest.l2 = int(vel1)
                velRequest.r1 = int(vel2)
                velRequest.r2 = int(vel2)
                velRequest.timetodrive = DELAY

            #if right trigger is a non zero val, move forwards
            elif rT:
                vel = abs(float(ic.linvel_calc(rT)))
                velRequest.l1 = int(vel)
                velRequest.l2 = int(vel)
                velRequest.r1 = int(vel)
                velRequest.r2 = int(vel)
                velRequest.timetodrive = DELAY

            #if left trigger is non zero val, move backwards
            elif lT:
                vel = -1*abs(float(ic.linvel_calc(lT)))
                velRequest.l1 = int(vel)
                velRequest.l2 = int(vel)
                velRequest.r1 = int(vel)
                velRequest.r2 = int(vel)
                velRequest.timetodrive = DELAY
        print(f"Left Speed: {velRequest.l1}, Right Speed: {velRequest.r1}")
        future = self.velsrv.call_async(velRequest)

        if future.result() is not None:
            self.get_logger().info("success")
        else:
            self.get_logger().info("fail?")
        return

    def destroy_node(self):
        self.controller.close()
        self.get_logger().info("Controller Node Closed")
        super().destroy_node()
    
def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()




        

