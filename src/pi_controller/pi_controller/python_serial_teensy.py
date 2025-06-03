import rclpy
from rclpy.node import Node
from rover_interfaces.msg import TelemData
from rover_interfaces.srv import RoverCommand
from pySerialTransfer import pySerialTransfer as tx
import time
import threading

COMPORT = '/dev/ttyACM0'

class Serial(Node):
    def __init__(self):
        super().__init__('Serial_Comm')
        self.serial_lock = threading.Lock()
        #declare parameters for ros node
        self.declare_parameter('COMPORT', '/dev/ttyACM0')
        self.declare_parameter('BaudRate', 38400)

        COMPORT = self.get_parameter('COMPORT').get_parameter_value().string_value
        BAUD = self.get_parameter('BaudRate').get_parameter_value().integer_value
        COMPORT = '/dev/ttyACM0'

        self.link = tx.SerialTransfer(COMPORT, BAUD) #set pySerialTransfer on COMPORT defined above

        try:
            self.link.open()
            time.sleep(6)
            self.get_logger().info("Serial connection opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.publisher = self.create_publisher(TelemData, 'Telemetry', 20)
        self.timer = self.create_timer(0.5, self.read_serial)
        self.srv = self.create_service(RoverCommand, 'RoverSerialCommand', self.send_command_callback)

    def read_serial(self):
        with self.serial_lock:
            if self.link.available():
                try:
                    telemOutput = []
                    for i in range(23): #get each value from telemetry serial
                            val = self.link.rx_obj(obj_type='f', start_pos=i*4)
                            telemOutput.append(val)
                except Exception as e:
                    self.get_logger().error(f"Error with serial read: {e}")
                    return
                
                data = TelemData()
                data.enc1 = telemOutput[0]
                data.enc2 = telemOutput[1]
                data.enc3 = telemOutput[2]
                data.enc4 = telemOutput[3]
                data.vel1 = telemOutput[4]
                data.vel2 = telemOutput[5]
                data.vel3 = telemOutput[6]
                data.vel4 = telemOutput[7]

                data.m1current = telemOutput[8]
                data.m2current = telemOutput[9]
                data.m3current = telemOutput[10]
                data.m4current = telemOutput[11]

                data.heading_x = telemOutput[12]
                data.heading_y = telemOutput[13]
                data.heading_z = telemOutput[14]

                data.heading_pos = telemOutput[15]
                data.heading_vel = telemOutput[16]

                data.accel_x = telemOutput[17]
                data.accel_y = telemOutput[18]
                data.accel_z = telemOutput[19]

                data.ang_accel_x = telemOutput[20]
                data.ang_accel_y = telemOutput[21]
                data.ang_accel_z = telemOutput[22]

                self.publisher.publish(data)
                self.get_logger().info("Published telem data")
                return
            else:
                self.get_logger().info("Serial not available")
                return

    def send_command_callback(self, request, response):
        self.get_logger().info("Sending data to Arduino")
        datasize = 0
        header = request.type[0]
        self.get_logger().info(f"header: {header}")
        try:
            with self.serial_lock:
                datasize = self.link.tx_obj(header, start_pos=datasize, val_type_override='c')
                for data in request.data:
                    self.get_logger().info(f"data: {data}")
                    datasize = self.link.tx_obj(data, start_pos=datasize, val_type_override='f')
                print(self.link.send(datasize))
                response.success = True
                self.get_logger().info(f"data sent")
        except Exception as e:
            self.get_logger().error(f"Error sending telem: {e}")
            response = False
        self.get_logger().info(f"responded")
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
