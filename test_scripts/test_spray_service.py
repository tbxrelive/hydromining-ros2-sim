#!/usr/bin/env python3
"""Quick client for the current /spray_action contract; requires an active node."""
import sys
import rclpy
from rclpy.node import Node
from hydromining_robot.srv import SprayAction


def main():
    rclpy.init()
    node=Node('spray_service_test')
    try:
        client=node.create_client(SprayAction,'/spray_action')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('No active /spray_action server')
        request=SprayAction.Request(use_tcp=True,pressure=20.,duration=12.,enabled=True)
        future=client.call_async(request)
        rclpy.spin_until_future_complete(node,future,timeout_sec=90)
        if not future.done():
            raise TimeoutError('Spray service timed out')
        response=future.result()
        print(response)
        if not response.success:
            raise RuntimeError(response.message)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    sys.exit(main())
