#!/usr/bin/env python3
import asyncio
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from rclpy.client import Client


class TestNode1(Node):
    def __init__(self):
        super().__init__('test_node_1')
        self.counter = 0
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            # If main() hasn't started the loop yet, get the default
            self.loop = asyncio.get_event_loop()
        
        # Reentrant group allows multiple callbacks to run in parallel
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.client_cbg = ReentrantCallbackGroup()
        if not self._initialize_services_clients():
            self.get_logger().error("Timeout while waiting for service")
            exit(-1)
        
        self._initialize_service_servers()
        self.get_logger().info("Scenario Manager Minimal is ready.")
   
    
    def _initialize_services_clients(self, service_wait_timeout: int| None = None) -> bool:
        self.trigger_cli = self.create_client(
            Trigger, "test_node_2/trigger_me", callback_group=self.client_cbg)
        
        for cli in [
            self.trigger_cli
        ]:
            self.get_logger().info(f"Waiting for {cli.service_name} to become ready.")
            if not cli.wait_for_service(service_wait_timeout):
                self.get_logger().warning(f"Timeout reached while waiting for {cli.service_name}")
                return False
            self.get_logger().info(f"{cli.service_name} ready.")
            
        return True
                
    def _initialize_service_servers(self):
        self.srv = self.create_service(
            Trigger, '~/trigger_me', self._load_scenario_cb, 
            callback_group=self.group1
        )
        self.srv2 = self.create_service(
            Trigger, '~/trigger_me2', self._load_scenario_cb2, 
            callback_group=self.group2
        )

    def _load_scenario_cb2(self, request, response):
        self.get_logger().info(f"Processing request for trigger service.")
        self.counter += 1


        response.message = f"I am so triggered right now, counter = {self.counter}"
        response.success = True
        return response

    def _load_scenario_cb(self, request, response):
        self.get_logger().info(f"Processing request for trigger service.")
        self.counter += 1

        trigger_req = Trigger.Request()
        result = self._call_service_with_timeout(self.trigger_cli, trigger_req)
        # alternative make function async and call like: result = await self.trigger_cli.call_async(trigger_req)
        
        if result is None:
            self.get_logger().error("Service call timed out.")
            response.message = "Service call timed out."
            response.success = False
            return response

        print(result)
        # Fill response to satisfy the interface
        response.message = f"I am so triggered right now, counter = {self.counter}"
        response.success = True
        return response
    
    def _call_service_with_timeout(self, client: Client, request: Client.Request, timeout: float = 5.0) -> Client.Response | None:    
        ros_future = client.call_async(request)
        event = threading.Event()
        
        def on_timeout():
            if not ros_future.done():
                client.remove_pending_request(ros_future)
                event.set()

        timer = threading.Timer(timeout, on_timeout)
        timer.start()

        # Success callback
        ros_future.add_done_callback(lambda _: (timer.cancel(), event.set()))

        # Block until either the timer or the result sets the event
        event.wait()
        return ros_future.result() if ros_future.done() else None

    # async def _call_and_wait(self, client: Client, request, timeout: float | None = 5.0):
    #         # 2. Use the CAPTURED loop, don't try to "get_running_loop" in a ROS thread
    #         asyncio_future = self.loop.create_future()
    #         ros_future = client.call_async(request)

    #         def ros_callback(f):
    #             # 3. Use call_soon_threadsafe to jump back to the Main Thread
    #             self.loop.call_soon_threadsafe(
    #                 lambda: not asyncio_future.done() and asyncio_future.set_result(f.result())
    #             )

    #         ros_future.add_done_callback(ros_callback)

    #         try:
    #             return await asyncio.wait_for(asyncio_future, timeout=timeout)
    #         except (asyncio.TimeoutError, asyncio.CancelledError):
    #             if not ros_future.done():
    #                 client.remove_pending_request(ros_future)
    #             return None
def main():
    rclpy.init()
    node = TestNode1()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()


##### NODE 2#########

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

class TestNode2(Node):
    def __init__(self):
        super().__init__('test_node_2')
        self.counter = 0
        
        # Reentrant group allows multiple callbacks to run in parallel
        self.group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            Trigger, 
            '~/trigger_me', 
            self._load_scenario_cb,
            callback_group=self.group
        )
        self.get_logger().info("Scenario Manager Minimal is ready.")

    def _load_scenario_cb(self, request, response):
        self.get_logger().info(f"Processing request for trigger service.")
        self.counter += 1

        #self._call_and_wait(self.trigger_cli, trigger_req)
        # Fill response to satisfy the interface
        response.message = f"I am so triggered right now, counter = {self.counter}"
        response.success = True
        return response

def main():
    rclpy.init()
    node = TestNode2()
    
    # Use MultiThreadedExecutor to ensure the node can handle 
    # new requests even if one is already active
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()