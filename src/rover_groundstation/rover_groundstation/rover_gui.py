import rclpy
from rclpy.node import Node

from nicegui import ui



#######
#ROS Node
#######


class gui(Node):
    def __init__(self):
        super().__init__("gui_viewer")

    def clicked(self):
        self.get_logger().info("button clicked")
















######
#BEGIN GUI
######


def gui_boot(node):
    with ui.card():
        ui.label("Test Card")
        ui.button('Test But', on_click=node.clicked())
    ui.run(port=5000)


def main(args=None):
    rclpy.init(args=args)
    node = gui()
    gui_boot(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()





