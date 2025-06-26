import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rover_interfaces.srv import BagStart
from std_srvs.srv import Trigger

import rosbag2_py
import importlib

class IMUBagger(Node):
    def __init__(self):
        super().__init__("universal_bagger_node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic_name', 'RoverTelem/roboclaw/enc_telem'),
                ("msg_type", 'rover_interfaces.msg.MotorData'),
                ('serialization_format', 'cdr'),
            ]
        )
        
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.msg_type_str = self.get_parameter('msg_type').get_parameter_value().string_value
        self.ser_format = self.get_parameter('serialization_format').get_parameter_value().string_value


        pkg, msg = self.msg_type_str.split(".msg.")
        msg_module = importlib.import_module(f"{pkg}.msg")
        self.msg_type = getattr(msg_module, msg)


        self.writer = rosbag2_py.SequentialWriter()

        self.topic_info = rosbag2_py.TopicMetadata(
            id=0,
            name=self.topic_name,
            type=self.msg_type_str.replace(".", "/"),
            serialization_format=self.ser_format)
        

        self.subscription = self.create_subscription(
            self.msg_type,
            self.topic_name,
            self.topic_callback,
            10)
        

        self.start_serv = self.create_service(BagStart, self.get_name() + "/start",self.start_bag_callback)
        self.stop_serv = self.create_service(Trigger, self.get_name() + "/stop", self.stop_bag_callback)

        self.lock = True


    def start_bag_callback(self, request, response):
        storage_options = rosbag2_py.StorageOptions(
            uri=request.uri + "/" + self.get_name(),
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        self.writer.create_topic(self.topic_info)
        self.lock = False
        self.get_logger().info(f"{self.get_name()} started bagging")
        return response

    def stop_bag_callback(self, request, response):
        self.writer.close()
        response.success = True
        self.lock = True
        self.get_logger().info(f"{self.get_name()} stopped bagging")
        return response
    


    def topic_callback(self, msg):
        if not self.lock:
            self.writer.write(
                self.topic_name,
                serialize_message(msg),
                self.get_clock().now().nanoseconds)
            self.get_logger().info("bagged msg")

    def destroy_node(self):
        self.writer.close()
        return super().destroy_node()
            
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

