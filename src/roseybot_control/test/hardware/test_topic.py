import rclpy
from rclpy.node import Node

import time
import pytest

topics = ["/cmd_vel", "/dynamic_joint_states", "/joint_state_broadcaster/transition_event",
          "/joint_states", "/roseybot_base_controller/cmd_vel_out", 
          "/roseybot_base_controller/odom", "/roseybot_base_controller/transition_event",
          "/tf"]
system_topics = ["/rosout", "/parameter_events"]

@pytest.mark.hardware
def test_topics():
    rclpy.init()

    # create simple ros node to listen for topics
    node = rclpy.create_node("_topic_listener")
    # wait for non system topics for 10 seconds
    timeout = 10
    wait = time.time()

    avail_topics = []

    try:

        while rclpy.ok():
            topic_types = node.get_topic_names_and_types()
            avail_topics = [name for name, _ in topic_types
                            if name not in system_topics]
            
            if avail_topics:
                break
            
            if (time.time() - wait) > timeout:
                break
            
        missing_topics = []

        if avail_topics:
            for name in topics:
                if name not in avail_topics:
                    missing_topics.append(name)

        assert missing_topics, f'Available Topics: {avail_topics}'

    finally:
        node.destroy_node()
        rclpy.shutdown()
    













    



