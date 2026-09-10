#!/usr/bin/env python3
"""
Dynamic ROS2 domain bridge for two rovers.

  Pi 1  (domain 1, ROS_DOMAIN_ID=1) — rover Pi on 192.168.2.x subnet:
      Main→Pi1 : /cmd_vel forwarded
      Pi1→Main : ALL topics discovered and forwarded

  Pi 2  (domain 2, ROS_DOMAIN_ID=2) — second rover Pi on separate subnet:
      Main→Pi2 : nothing sent
      Pi2→Main : ALL topics discovered and forwarded

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
DOMAIN_PI1  = 1   # rover Pi 1 (cmd_vel forwarded to it)
DOMAIN_PI2  = 2   # rover Pi 2 (listen-only, nothing sent)

SKIP_TOPICS = {'/parameter_events', '/rosout', '/cmd_vel', '/clock'}


def qos_for(topic: str) -> QoSProfile:
    if topic == '/tf_static':
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=100)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE, depth=10)


class PiListener:
    """
    Bridges one Pi domain → main domain.
    Discovers all published topics on the Pi domain every 2 s and
    forwards them to domain 0.  Nothing is ever sent back to this Pi.
    """
    def __init__(self, node_pi: Node, node_main: Node,
                 domain_id: int, label: str, bridged_set: set, lock: threading.Lock):
        self._n_pi   = node_pi
        self._n_main = node_main
        self._label  = label
        self._bridged = bridged_set
        self._lock    = lock
        node_pi.create_timer(2.0, self._discover)

    def _discover(self):
        for topic, types in self._n_pi.get_topic_names_and_types():
            if topic in SKIP_TOPICS or not types:
                continue
            if self._n_pi.count_publishers(topic) == 0:
                continue
            with self._lock:
                key = (self._label, topic)
                if key in self._bridged:
                    continue
                type_str = types[0]
                qos = qos_for(topic)
                try:
                    msg_class = get_message(type_str)
                    pub = self._n_main.create_publisher(msg_class, topic, qos)
                    def cb(msg, p=pub):
                        p.publish(msg)
                    self._n_pi.create_subscription(msg_class, topic, cb, qos)
                    self._bridged.add(key)
                    print(f'[bridge] {self._label}→Main: {topic}  [{type_str}]')
                except Exception as e:
                    print(f'[bridge] FAILED {self._label} {topic}: {e}')


class DualBridge:
    def __init__(self):
        # ── Contexts & nodes ────────────────────────────────────────────────
        self.ctx0 = Context(); rclpy.init(context=self.ctx0, domain_id=DOMAIN_MAIN)
        self.ctx1 = Context(); rclpy.init(context=self.ctx1, domain_id=DOMAIN_PI1)
        self.ctx2 = Context(); rclpy.init(context=self.ctx2, domain_id=DOMAIN_PI2)

        self.n0 = Node('bridge_main', context=self.ctx0)
        self.n1 = Node('bridge_pi1',  context=self.ctx1)
        self.n2 = Node('bridge_pi2',  context=self.ctx2)

        self.bridged: set = set()
        self.lock = threading.Lock()

        # ── Main→Pi1: /cmd_vel ───────────────────────────────────────────────
        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)
        pub_cmd = self.n1.create_publisher(TwistStamped, '/cmd_vel', qos_cmd)
        self.n0.create_subscription(TwistStamped, '/cmd_vel', pub_cmd.publish, qos_cmd)
        print('[bridge] Main→Pi1: /cmd_vel [geometry_msgs/msg/TwistStamped]')
        print('[bridge] Main→Pi2: (nothing forwarded)')

        # ── Pi1→Main and Pi2→Main: auto-discover ────────────────────────────
        PiListener(self.n1, self.n0, DOMAIN_PI1, 'Pi1', self.bridged, self.lock)
        PiListener(self.n2, self.n0, DOMAIN_PI2, 'Pi2', self.bridged, self.lock)

        # ── Executors ───────────────────────────────────────────────────────
        self.exec0 = MultiThreadedExecutor(context=self.ctx0)
        self.exec0.add_node(self.n0)
        self.exec1 = MultiThreadedExecutor(context=self.ctx1)
        self.exec1.add_node(self.n1)
        self.exec2 = MultiThreadedExecutor(context=self.ctx2)
        self.exec2.add_node(self.n2)

    def spin(self):
        threads = [
            threading.Thread(target=self.exec0.spin, daemon=True),
            threading.Thread(target=self.exec1.spin, daemon=True),
            threading.Thread(target=self.exec2.spin, daemon=True),
        ]
        for t in threads:
            t.start()
        try:
            for t in threads:
                t.join()
        except KeyboardInterrupt:
            pass
        finally:
            self.exec0.shutdown(); self.exec1.shutdown(); self.exec2.shutdown()
            rclpy.shutdown(context=self.ctx0)
            rclpy.shutdown(context=self.ctx1)
            rclpy.shutdown(context=self.ctx2)


if __name__ == '__main__':
    DualBridge().spin()
