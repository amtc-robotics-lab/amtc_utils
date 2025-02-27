#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from autonomous_machine_interface.msg import Heartbeat


class NeroResourceHeartbeat:

    def __init__(
            self,
            node: Node
    ) -> None:
        self.node = node
        self.seq = 0
        self.configure()
        self.activate()

    def configure(self) -> None:
        self.hearbeat_period = self.node.declare_parameter('RESOURCE.hearbeat_period', Parameter.Type.DOUBLE).value
        pass

    def activate(self) -> None:
        self.publisher = self.node.create_publisher(Heartbeat, self.node.get_name() + '/heartbeat', 10)
        self.hearbet_timer = self.node.create_timer(self.hearbeat_period, self.hearbeat_callback)
        self.node.get_logger().info('Heartbeat created')
        pass

    def hearbeat_callback(self) -> None:
        msg = Heartbeat()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.seq = self.seq
        msg.node_name = self.node.get_name()
        self.seq += 1
        self.publisher.publish(msg)
        pass