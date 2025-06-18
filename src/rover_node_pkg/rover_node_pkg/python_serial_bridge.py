import rclpy
from rclpy.node import Node
import rclpy.parameter
from rover_interfaces.msg import MotorData
from rover_interfaces.srv import RoverCommand
from pySerialTransfer import pySerialTransfer as tx
import time
import threading


class Serial(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.serial_lock = threading.Lock()

        #declare parameters for ros node
        self.declare_parameters(
            namespace='',
            parameters=[
                ('COMPORT', "/dev/ttyACM0"),
                ('BAUD', 115200),
                ('PREFIX', "RoverTelem/roboclaw/"),
            ]
        )

        #get parameters
        COMPORT = self.get_parameter('COMPORT').value
        BAUD = self.get_parameter('BAUD').value
        PREFIX = self.get_parameter('PREFIX').value

        try:
            self.link = tx.SerialTransfer(COMPORT, BAUD) #set pySerialTransfer on COMPORT defined above
            self.link.open()
            time.sleep(0.1)
            self.get_logger().info("Serial connection opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to start node: {e}")
            rclpy.shutdown()
            return

        #create publisher, service, and timer to publish data
        self.encPub = self.create_publisher(MotorData, PREFIX + 'Enc_Telem', 20)
        self.timer = self.create_timer(0.025, self.read_serial)
        self.srv = self.create_service(RoverCommand, 'SerialCommand', self.send_command_callback)

    def read_serial(self):
        with self.serial_lock:
            if self.link.available():
                try:
                    telemOutput = []
                    for i in range(12): #get each value from telemetry serial
                            val = self.link.rx_obj(obj_type='i', start_pos=i*4)
                            telemOutput.append(val)
                except Exception as e:
                    self.get_logger().error(f"Error with serial read: {e}")
                    return

                motorData = MotorData()
                motorData.enc1 = telemOutput[0]
                motorData.enc2 = telemOutput[1]
                motorData.enc3 = telemOutput[2]
                motorData.enc4 = telemOutput[3]

                motorData.vel1 = telemOutput[4]
                motorData.vel2 = telemOutput[5]
                motorData.vel3 = telemOutput[6]
                motorData.vel4 = telemOutput[7]

                motorData.m1current = telemOutput[8]
                motorData.m2current = telemOutput[9]
                motorData.m3current = telemOutput[10]
                motorData.m4current = telemOutput[11]
                self.encPub.publish(motorData)
                return
            else:
                self.get_logger.info("serial not available?")
                return

    def send_command_callback(self, request, response):
        datasize = 0
        header = request.type[0]
        try:
            with self.serial_lock:
                datasize = self.link.tx_obj(header, start_pos=datasize, val_type_override='c')
                for data in request.data:
                    try:
                        datasize = self.link.tx_obj(data, start_pos=datasize, val_type_override='i')
                    except Exception as e:
                        self.get_logger().error(f"Error adding data: {e}")
                self.get_logger().info("sent command to teensy")
                self.link.send(datasize)
                response.success = True
        except Exception as e:
            self.get_logger().error(f"Error sending telem: {e}")
            response = False
        return response


    def destroy_node(self):
        self.link.close()
        self.get_logger().info("Serial connection closed")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Serial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
