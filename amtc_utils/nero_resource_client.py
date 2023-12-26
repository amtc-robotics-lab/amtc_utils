#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from resource_manager_msgs.srv._alloc import Alloc
from resource_manager_msgs.srv._free import Free
from resource_manager_msgs.msg import Token


class NeroResourceClient:

    def __init__(
            self,
            node: Node,
            resource_type: str,
            callback_group: MutuallyExclusiveCallbackGroup
    ) -> None:
        self.node = node
        self.resource_type = resource_type
        self.access_data = Token()
        self._srv_call_cbg = callback_group

        # Namespace
        self._namespace = node.get_namespace()

        # Service clients
        self._cli_alloc_resource = node.create_client(srv_type=Alloc,
                                                      srv_name='resource_manager/alloc',
                                                      callback_group=self._srv_call_cbg)

        self._cli_free_resource = node.create_client(srv_type=Free,
                                                     srv_name='resource_manager/free',
                                                     callback_group=self._srv_call_cbg)

    def alloc(
            self,
            timeout: float
    ) -> bool:
        """
        Calls resource manager to allocate resource named during object creation.

        :param timeout: Time to wait for allocation service.
        :type timeout: float
        :return: True if resource successfully allocated, False else.
        :rtype: bool
        """

        alloc_req = Alloc.Request()
        alloc_req.resource.type = self.resource_type

        if not self._cli_alloc_resource.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().info('[RESOURCE {0}] Failed because alloc server timeout...')
            return False

        future = self._cli_alloc_resource.call_async(alloc_req)
        self.node.executor.spin_until_future_complete(future)

        if future.result().is_success is False:
            self.node.get_parameter('[RESOURCE {0}] Failed to allocate resource {0}'.format(self.resource_type))
            return False

        self.access_data.data = future.result().token.data
        return True

    def free(
            self,
            timeout: float
    ) -> bool:
        """
        Calls resource manager to free resource named during object creation.

        :param timeout: Time to wait for freeing service.
        :type timeout: float
        :return: True if resource successfully freed, False else.
        :rtype: bool
        """

        free_req = Free.Request()
        free_req.token.data = self.access_data.data

        if not self._cli_free_resource.wait_for_service(timeout_sec=timeout):
            self.node.get_logger().info('[RESOURCE {0}] Failed because free server timeout...')
            return False

        future = self._cli_free_resource.call_async(free_req)
        self.node.executor.spin_until_future_complete(future)

        if future.result().is_success is False:
            self.node.get_parameter('[RESOURCE {0}] Failed to free resource {0}'.format(self.resource_type))
            return False

        self.access_data.data = ""
        return True
