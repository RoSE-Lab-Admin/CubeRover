import rclpy
from rclpy import Node

class MultiPlot(Node):
    def __init__(self):
        super().__init__("plot_updater")

    