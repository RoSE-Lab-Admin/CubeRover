import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rover_interfaces.msg import MotorData
from rover_interfaces.srv import BagStart
from std_srvs import Trigger

import rosbag2_py

class IMUBagger(Node):
    def __init__(self):
        super().__init__("motor_bagger_node")
        self.writer = rosbag2_py.SequentialWriter()

        self.start_serv = self.create_service(BagStart,"motor_bag_start",self.start_bag_callback)
        self.stop_serv = self.create_service(Trigger, "motor_bag_stop", self.stop_bag_callback)
        self.topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name='RoverTelem/roboclaw/enc_telem',
            type='rover_interfaces/msg/MotorData',
            serialization_format='cdr')
        

        self.subscription = self.create_subscription(
            MotorData,
            'RoverTelem/roboclaw/enc_telem',
            self.topic_callback,
            10)
        
        self.lock = True


    def start_bag_callback(self, request, response):
        storage_options = rosbag2_py.StorageOptions(
            uri=request.uri,
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        self.writer.create_topic(self.topic_info)
        self.lock = False
        return response

    def stop_bag_callback(self, request, response):
        self.writer.close()
        response.success = True
        self.lock = True
        return response
    
    def destroy_node(self):
        return super().destroy_node()


    def topic_callback(self, msg):
        if not self.lock:
            self.writer.write(
                'RoverTelem/roboclaw/enc_telem',
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
            
def main(args=None):
    rclpy.init(args=args)
    node = IMUBagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

