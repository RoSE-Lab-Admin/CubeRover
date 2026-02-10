import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
class TestEndToEnd(Node):
    def __init__(self):
        super().__init__('test_end_to_end')

        # create publisher
        self.vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # create twist stamped message
        self.vel = TwistStamped()
        self.vel.header.stamp = self.get_clock().now().to_msg()
        self.vel.header.frame_id = 'base_link'
        self.vel.twist.linear.x = 1.0

        # create timer so it runs for 30 seconds
        self.timer = self.create_timer(0.05, self.publish_vel)
        self.start_time = self.get_clock().now().nanoseconds

    def publish_vel(self):
        run_time = 30 * 1_000_000_000
        if (self.get_clock().now().nanoseconds - self.start_time) > run_time:
            self.timer.cancel()
            rclpy.shutdown()
            return
        else:
            self.vel.header.stamp = self.get_clock().now().to_msg()
            self.vel_pub.publish(self.vel)

def main():
    rclpy.init()
    node = TestEndToEnd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()