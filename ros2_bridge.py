#!/usr/bin/env python3
"""
Dynamic ROS2 domain bridge.
  Pi→Main  : auto-discovers ALL topics published on domain 1 (Pi) and forwards them to domain 0
  Main→Pi  : forwards /cmd_vel from domain 0 to domain 1
Run on the workstation before launching Nav2.
"""
import threading
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
from geometry_msgs.msg import TwistStamped

DOMAIN_MAIN = 0   # workstation / NUC
DOMAIN_PI   = 1   # rover Pi

# Topics to never bridge from Pi→Main (internal ROS2 overhead or would create loops)
SKIP_TOPICS = {'/parameter_events', '/rosout', '/cmd_vel', '/clock'}

def qos_for(topic: str) -> QoSProfile:
    if topic == '/tf_static':
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=100)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE, depth=10)


class DynamicBridge:
    def __init__(self):
        self.ctx0 = Context()
        rclpy.init(context=self.ctx0, domain_id=DOMAIN_MAIN)
        self.ctx1 = Context()
        rclpy.init(context=self.ctx1, domain_id=DOMAIN_PI)

        self.n0 = Node('bridge_main', context=self.ctx0)
        self.n1 = Node('bridge_pi',   context=self.ctx1)

        self.bridged: set = set()
        self.lock = threading.Lock()

        # Main→Pi: /cmd_vel (only topic the Pi needs from the workstation)
        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)
        pub_cmd = self.n1.create_publisher(TwistStamped, '/cmd_vel', qos_cmd)
        self.n0.create_subscription(TwistStamped, '/cmd_vel', pub_cmd.publish, qos_cmd)
        print('[bridge] Main→Pi: /cmd_vel [geometry_msgs/msg/TwistStamped]')

        # Timer: discover and bridge new Pi topics every 2 s
        self.n1.create_timer(2.0, self._discover)

        self.exec0 = MultiThreadedExecutor(context=self.ctx0)
        self.exec0.add_node(self.n0)
        self.exec1 = MultiThreadedExecutor(context=self.ctx1)
        self.exec1.add_node(self.n1)

    def _discover(self):
        for topic, types in self.n1.get_topic_names_and_types():
            if topic in SKIP_TOPICS or not types:
                continue
            # Only bridge topics that actually have a publisher on the Pi side
            if self.n1.count_publishers(topic) == 0:
                continue
            with self.lock:
                if topic in self.bridged:
                    continue
                type_str = types[0]
                qos = qos_for(topic)
                try:
                    msg_class = get_message(type_str)
                    pub = self.n0.create_publisher(msg_class, topic, qos)
                    def cb(msg, p=pub):
                        p.publish(msg)
                    self.n1.create_subscription(msg_class, topic, cb, qos)
                    self.bridged.add(topic)
                    print(f'[bridge] Pi→Main: {topic}  [{type_str}]')
                except Exception as e:
                    print(f'[bridge] FAILED {topic}: {e}')

    def spin(self):
        t0 = threading.Thread(target=self.exec0.spin, daemon=True)
        t1 = threading.Thread(target=self.exec1.spin, daemon=True)
        t0.start()
        t1.start()
        try:
            t0.join()
            t1.join()
        except KeyboardInterrupt:
            pass
        finally:
            self.exec0.shutdown()
            self.exec1.shutdown()
            rclpy.shutdown(context=self.ctx0)
            rclpy.shutdown(context=self.ctx1)


if __name__ == '__main__':
    DynamicBridge().spin()