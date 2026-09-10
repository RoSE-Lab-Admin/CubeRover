#!/usr/bin/env python3
"""
Dynamic ROS2 domain bridge for two rovers.

  Pi 1  (domain 1, ROS_DOMAIN_ID=1):
      Main→Pi1 : /cmd_vel forwarded
      Pi1→Main : ALL topics discovered and forwarded

  Pi 2  (domain 2, ROS_DOMAIN_ID=2):
      Main→Pi2 : /cmd_vel forwarded (needed for DDS bidirectional discovery)
      Pi2→Main : ALL topics discovered and forwarded

Each Pi gets its own isolated pair of nodes so discovery never shares
mutable node state across executor threads.

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
DOMAIN_PI1  = 1   # rover Pi 1 (receives cmd_vel)
DOMAIN_PI2  = 2   # rover Pi 2 (listen-only)

SKIP_TOPICS = {'/parameter_events', '/rosout', '/cmd_vel', '/clock'}


def qos_for(topic: str) -> QoSProfile:
    if topic == '/tf_static':
        return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=100)
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE, depth=10)


class PiListener:
    """
    Discovers all topics published on node_pi's domain and forwards them to
    node_main.  node_main is a dedicated domain-0 node owned solely by this
    listener, so no state is shared with any other listener.
    """

    def __init__(self, node_pi: Node, node_main: Node, label: str,
                 extra_skip: set = None):
        self._n_pi   = node_pi
        self._n_main = node_main
        self._label  = label
        self._skip   = SKIP_TOPICS | (extra_skip or set())
        self._bridged: set = set()
        node_pi.create_timer(2.0, self._discover)

    def _discover(self):
        for topic, types in self._n_pi.get_topic_names_and_types():
            if topic in self._skip or not types:
                continue
            if self._n_pi.count_publishers(topic) == 0:
                continue
            if topic in self._bridged:
                continue
            type_str = types[0]
            qos = qos_for(topic)
            try:
                msg_class = get_message(type_str)
                pub = self._n_main.create_publisher(msg_class, topic, qos)
                def cb(msg, p=pub):
                    p.publish(msg)
                self._n_pi.create_subscription(msg_class, topic, cb, qos)
                self._bridged.add(topic)
                print(f'[bridge] {self._label}→Main: {topic}  [{type_str}]')
            except Exception as e:
                print(f'[bridge] FAILED {self._label} {topic}: {e}')


class DualBridge:
    def __init__(self):
        # ── Contexts ────────────────────────────────────────────────────────
        self.ctx0 = Context(); rclpy.init(context=self.ctx0, domain_id=DOMAIN_MAIN)
        self.ctx1 = Context(); rclpy.init(context=self.ctx1, domain_id=DOMAIN_PI1)
        self.ctx2 = Context(); rclpy.init(context=self.ctx2, domain_id=DOMAIN_PI2)

        # ── Nodes ───────────────────────────────────────────────────────────
        # Domain 0: one control node (cmd_vel) + one forward node per Pi
        self.n0_ctrl = Node('bridge_main_ctrl',    context=self.ctx0)
        self.n0_pi1  = Node('bridge_main_from_pi1', context=self.ctx0)
        self.n0_pi2  = Node('bridge_main_from_pi2', context=self.ctx0)
        # Domain 1 and 2: one discovery/subscriber node each
        self.n1      = Node('bridge_pi1',           context=self.ctx1)
        self.n2      = Node('bridge_pi2',           context=self.ctx2)

        # ── Main→Pi1 and Pi2: /cmd_vel ──────────────────────────────────────
        # Forwarding cmd_vel to Pi2 as well ensures bridge_pi2 is a real DDS
        # publisher on domain 2, which is required for bidirectional discovery.
        qos_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE, depth=10)
        pub_cmd1 = self.n1.create_publisher(TwistStamped, '/cmd_vel', qos_cmd)
        pub_cmd2 = self.n2.create_publisher(TwistStamped, '/cmd_vel', qos_cmd)

        def forward_cmd_vel(msg):
            pub_cmd1.publish(msg)
            pub_cmd2.publish(msg)

        self.n0_ctrl.create_subscription(TwistStamped, '/cmd_vel', forward_cmd_vel, qos_cmd)
        print('[bridge] Main→Pi1: /cmd_vel [geometry_msgs/msg/TwistStamped]')
        print('[bridge] Main→Pi2: /cmd_vel [geometry_msgs/msg/TwistStamped]')

        # ── Pi→Main: each listener owns its own main-side node ───────────────
        PI2_SKIP = {
            '/MastCam/Front/color/image_raw/compressedDepth',
            '/MastCam/Front/depth/image_rect_raw/compressed',
        }
        PiListener(self.n1, self.n0_pi1, 'Pi1')
        PiListener(self.n2, self.n0_pi2, 'Pi2', extra_skip=PI2_SKIP)

        # ── Executors ───────────────────────────────────────────────────────
        self.exec0 = MultiThreadedExecutor(context=self.ctx0)
        for node in (self.n0_ctrl, self.n0_pi1, self.n0_pi2):
            self.exec0.add_node(node)

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
            self.exec0.shutdown()
            self.exec1.shutdown()
            self.exec2.shutdown()
            rclpy.shutdown(context=self.ctx0)
            rclpy.shutdown(context=self.ctx1)
            rclpy.shutdown(context=self.ctx2)


if __name__ == '__main__':
    DualBridge().spin()
