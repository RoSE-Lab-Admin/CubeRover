import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class CovarianceRelay(Node):
    def __init__(self):
        super().__init__('covariance_relay')

        # create odometry covariance values
        self.pose_cov = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
        self.twist_cov = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]

        # subscribe to odometry topic with no covariance
        self.odom_sub = self.create_subscription(Odometry, 
                                                 '/roseybot_base_controller/odom_raw',
                                                 self.odom_callback, 10)
        # publish to odometry topic ekf expects
        self.odom_pub = self.create_publisher(Odometry, '/roseybot_base_controller/odom',
                                              10)
        
    def odom_callback(self, msg):
        for i in range(6):
            # set diagnonal values to covariance values
            msg.pose.covariance[i * 7] = self.pose_cov[i]
            msg.twist.covariance[i * 7] = self.twist_cov[i]

        # sends back modified odometry data
        self.odom_pub.publish(msg)
    
def main():
    rclpy.init()
    relay = CovarianceRelay()
    rclpy.spin(relay)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
