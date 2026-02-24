#!/usr/bin/env python3

# Copyright 2026 Manuel Muth
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from typing import Any
from time import sleep

import threading
import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.client import Client
from rclpy.task import Future
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange

from rosiris_manipulation_interfaces.srv import (
    AddCollisionObjects,
    AttachCollisionObject,
    DetachCollisionObject,
    GetAttachedCollisionObjectIds,
    GetCollisionObjectIds,
    LoadScenario,
    MoveCollisionObjects,
    RemoveCollisionObjects,
    ResetScenario,
    UpdateAllowedCollisions,
)
from rosiris_manipulation_interfaces.msg import (
    ServiceErrorCode,
    ServiceResult,
)

from rosiris_manipulation_utils.scenario_loader import LoadScenarioError
from rosiris_manipulation_utils.scenario_instance import ScenarioInstance
from rosiris_manipulation_utils.scenario_loader import NoSuitableLoaderError

class ScenarioManager(Node):

    def __init__(self, node_name: str = "scenario_manager"):
        super().__init__(node_name)
        self.server_req_cbg = MutuallyExclusiveCallbackGroup()
        self.client_cbg = ReentrantCallbackGroup()
        self._scenario : ScenarioInstance| None = None

        self._declare_and_get_parameters()
        if not self._initialize_services_clients(self.wait_for_services_timeout):
            self.get_logger().error("Timeout while waiting for service")
            exit(-1)

        self._initialize_service_servers()

    def _declare_and_get_parameters(self):
        # wait_for_services_timeout
        wait_for_services_timeout_desrc = ParameterDescriptor(
            description='How long to wait for services to become ready in seconds, -1 = forever',
            type=Parameter.Type.INTEGER,
            integer_range=[
                IntegerRange(
                    from_value=-1,
                    to_value=2**63 - 1,
                    step=1
                )
            ]
        )
        # Only None is defined as wait forever in wait_for_service...
        self.wait_for_services_timeout: int | None = (
            None if (v := self.declare_parameter('wait_for_services_timeout', -1, wait_for_services_timeout_desrc).value) < 0 else v
        )
        self.get_logger().info(f"wait_for_services_timeout = {self.wait_for_services_timeout}")

        # service_call_timeout
        service_call_timeout_desrc = ParameterDescriptor(
            description='How long to wait for service calls to complete in milliseconds',
            type=Parameter.Type.INTEGER,
            integer_range=[
                IntegerRange(
                    from_value=0,
                    to_value=2**63 - 1,
                    step=1
                )
            ]
        )
        # Only None is defined as wait forever in wait_for_service...
        self.service_call_timeout: float = float(self.declare_parameter('service_call_timeout', 5000, service_call_timeout_desrc).value) / 1000.0
        self.get_logger().info(f"service_call_timeout = {self.service_call_timeout}")

        # path_to_scenario_file
        path_to_scenario_descr = ParameterDescriptor(
            description='Path to the scenario which should be loaded on startup.',
            type=Parameter.Type.STRING,
        )
        self._path_to_scenario_file: str = self.declare_parameter('path_to_scenario_file', "", path_to_scenario_descr).value
        self.get_logger().info(f"initial_scenario_path = {self._path_to_scenario_file}")


    def _initialize_services_clients(self, service_wait_timeout: int| None = None) -> bool:
        self.add_cli = self.create_client(
            AddCollisionObjects, "planning_scene_manager/add_collision_objects", callback_group=self.client_cbg)
        self.move_cli = self.create_client(
            MoveCollisionObjects, "planning_scene_manager/move_collision_objects", callback_group=self.client_cbg)
        self.attach_cli = self.create_client(
            AttachCollisionObject, "planning_scene_manager/attach_collision_object", callback_group=self.client_cbg)
        self.detach_cli = self.create_client(
            DetachCollisionObject, "planning_scene_manager/detach_collision_object", callback_group=self.client_cbg)
        self.remove_cli = self.create_client(
            RemoveCollisionObjects, "planning_scene_manager/remove_collision_objects", callback_group=self.client_cbg)
        self.acm_cli = self.create_client(
            UpdateAllowedCollisions, "planning_scene_manager/update_allowed_collisions", callback_group=self.client_cbg)
        self.get_attached_obj_ids_cli = self.create_client(
            GetAttachedCollisionObjectIds, "planning_scene_manager/get_attached_collision_obj_ids", callback_group=self.client_cbg)
        self.get_collision_obj_ids_cli = self.create_client(
            GetCollisionObjectIds, "planning_scene_manager/get_collision_obj_ids", callback_group=self.client_cbg)
        
        for cli in [
            self.add_cli, self.move_cli, self.attach_cli,
            self.detach_cli, self.remove_cli, self.acm_cli,
            self.get_attached_obj_ids_cli, self.get_collision_obj_ids_cli
        ]:
            self.get_logger().info(f"Waiting for {cli.service_name} to become ready.")
            if not cli.wait_for_service(service_wait_timeout):
                self.get_logger().warning(f"Timeout reached while waiting for {cli.service_name}")
                return False
            self.get_logger().info(f"{cli.service_name} ready.")
            
        return True
                
    def _initialize_service_servers(self):
        # both services are in same MutuallyExclusiveCallbackGroup to ensure that only one of them is executed at a time
        self.setup_scenario_srv = self.create_service(LoadScenario, '~/load_scenario', self._load_scenario_cb, callback_group=self.server_req_cbg)
        self.reset_scenario_srv = self.create_service(ResetScenario, '~/reset_scenario', self._reset_scenario_cb, callback_group=self.server_req_cbg)

    def load_initial_scenario(self):
        # load a initial scenario it one is passed via params
        if self._path_to_scenario_file:
            resp = self._load_scenario(self._path_to_scenario_file)
            if resp.result.return_type == ServiceResult.SUCCESS and  self._scenario is not None:
                self.get_logger().info(f"Started {self.get_name()} with initial scenario: {self._scenario.name}")
            else:
                self.get_logger().warning(f"Could not load initial scenario at: {self._path_to_scenario_file}")
                self.get_logger().warning(f"Got response: {resp.result.return_type}, with {resp.result.error_code}.")
                self.get_logger().warning(f"Error Message: {resp.result.message}")


    async def _load_scenario_cb(self, req: LoadScenario.Request, resp: LoadScenario.Response):
        scenario_path = req.path_to_scenario_file
        if not scenario_path or not scenario_path.strip():
            resp.result = self._srv_res(f"No path provided.", ServiceResult.ERROR, ServiceErrorCode.ERROR_VALUE)
            return resp
        
        if self._scenario is not None and self._scenario.loaded_in_scene:
            resp.result = self._srv_res(f"A scenario is already loaded.", ServiceResult.ERROR, ServiceErrorCode.ERROR_VALUE)
            return resp

        return self._load_scenario(scenario_path)

    
    def _load_scenario(self, path_to_scenario: str) -> LoadScenario.Response:
        resp = LoadScenario.Response()
        try:
            self._scenario = ScenarioInstance(path_to_scenario)
        except LoadScenarioError as e:
            msg = f"Loading of the scenario {path_to_scenario} failed: {e}"
            self.get_logger().error(msg)
            resp.result = self._srv_res(msg, ServiceResult.ERROR, ServiceErrorCode.ERROR_VALUE)
            return resp
        except NoSuitableLoaderError as e:
            msg = f"No loader found for file {path_to_scenario}: {e}"
            self.get_logger().error(msg)
            resp.result = self._srv_res(msg, ServiceResult.ERROR, ServiceErrorCode.ERROR_VALUE)
            return resp

        resp.result = self._load_scenario_into_scene(self._scenario)
        return resp
    
    def _load_scenario_into_scene(self, scenario: ScenarioInstance) -> ServiceResult:
        self.get_logger().info(f"Loading scenario into scene: {scenario.name}")
        objs_to_add, col_mtrx_upds = scenario.to_msg()
        # add collision obj from scenario:
        add_obj_req = AddCollisionObjects.Request()
        add_obj_req.collision_objects = objs_to_add
        add_cli_resp : AddCollisionObjects.Response | None = self._call_service_with_timeout(self.add_cli, add_obj_req, self.service_call_timeout)
        
        if add_cli_resp is None:
                return self._no_response_srv_res(self.add_cli)
        if add_cli_resp.result.return_type != ServiceResult.SUCCESS:
            return self._srv_res(f"Failed to load scenario: {add_cli_resp.result.message}", add_cli_resp.result.return_type, add_cli_resp.result.error_code)

        scenario.loaded_in_scene = True
        success_msg = f"Successful setup up the {scenario.name} scenario."
        self.get_logger().info(success_msg)
        return self._srv_res(success_msg, ServiceResult.SUCCESS, ServiceErrorCode.NO_ERROR)


    def _reset_scenario_cb(self, _: ResetScenario.Request, resp: ResetScenario.Response):
        if self._scenario is None or not self._scenario.loaded_in_scene:
            err_msg = "No scenario loaded, cannot reset."
            self.get_logger().warning(err_msg)
            resp.result = self._srv_res(err_msg, ServiceResult.ERROR, ServiceErrorCode.ERROR_NO_SCENARIO_LOADED)
            return resp
        # first we check for all attached objects
        detach_res = self._detach_all_objects()
        if detach_res.return_type != ServiceResult.SUCCESS:
            resp.result = detach_res
            return resp
        
        # then we remove all objects of the scenario
        # this resets the collision matrix as well
        remove_res = self._remove_all_objects()
        if remove_res.return_type != ServiceResult.SUCCESS:
            resp.result = remove_res
            return resp
        
        # finally we load the scenario again
        self.get_logger().info(f"Reloading scenario: {self._scenario.name}")
        resp.result = self._load_scenario_into_scene(self._scenario)
        return resp

    def _detach_all_objects(self) -> ServiceResult:
        self.get_logger().info("Detaching all objects from robot.")
        attached_obj_req = GetAttachedCollisionObjectIds.Request()
        attached_obj_resp : GetAttachedCollisionObjectIds.Response | None = self._call_service_with_timeout(self.get_attached_obj_ids_cli, attached_obj_req, self.service_call_timeout)
        if attached_obj_resp is None:
            return self._no_response_srv_res(self.get_attached_obj_ids_cli)
        if attached_obj_resp.result.return_type != ServiceResult.SUCCESS:
            msg = f"Failed to get attached objects: {attached_obj_resp.result.message}"
            self.get_logger().error(msg)
            return self._srv_res(msg, attached_obj_resp.result.return_type, attached_obj_resp.result.error_code)

        detach_req = DetachCollisionObject.Request()
        # get all ids that are part of the scenario, we only detach objects added by scenario manager
        for obj in attached_obj_resp.attached_objects:
            if obj.object.id in self._scenario.collision_object_ids:
                detach_req.detach_from_link = obj.link_name
                detach_req.collision_object_id = obj.object.id
                self.get_logger().info(f"Detaching object {obj.object.id} from link {obj.link_name}")
                detach_resp : DetachCollisionObject.Response | None = self._call_service_with_timeout(self.detach_cli, detach_req, self.service_call_timeout)
                if detach_resp is None: 
                    return self._no_response_srv_res(self.detach_cli)
                if detach_resp.result.return_type != ServiceResult.SUCCESS:
                    msg = f"Failed to detach object {obj.object.id}: {detach_resp.result.message}"
                    self.get_logger().error(msg)
                    return self._srv_res(msg, detach_resp.result.return_type, detach_resp.result.error_code)
        
        self.get_logger().info("Successfully detached all objects from robot.")
        return self._srv_res("Successfully detached all objects.", ServiceResult.SUCCESS, ServiceErrorCode.NO_ERROR)     
                       
    def _remove_all_objects(self) -> ServiceResult:
        self.get_logger().info("Removing all objects from scene.")
        col_obj_req = GetCollisionObjectIds.Request()
        objs_in_scene : GetCollisionObjectIds.Response | None = self._call_service_with_timeout(self.get_collision_obj_ids_cli, col_obj_req, self.service_call_timeout)
        if objs_in_scene is None:
            return self._no_response_srv_res(self.get_collision_obj_ids_cli)
        if objs_in_scene.result.return_type != ServiceResult.SUCCESS:
            msg = f"Failed to get collision objects: {objs_in_scene.result.message}"
            self.get_logger().error(msg)
            return self._srv_res(msg, objs_in_scene.result.return_type, objs_in_scene.result.error_code)
        
        remove_req = RemoveCollisionObjects.Request()
        # only remove objects that are part of the scenario, we do not want to remove objects added by other nodes
        remove_req.object_ids = [obj_id for obj_id in objs_in_scene.object_ids if obj_id in self._scenario.collision_object_ids]
        self.get_logger().info(f"Removing objects: {remove_req.object_ids} from scene.")
        remove_resp : RemoveCollisionObjects.Response | None = self._call_service_with_timeout(self.remove_cli, remove_req, self.service_call_timeout)
        if remove_resp is None:
            return self._no_response_srv_res(self.remove_cli)
        if remove_resp.result.return_type != ServiceResult.SUCCESS:
            msg = f"Failed to remove collision objects: {remove_resp.result.message}"
            self.get_logger().error(msg)
            return self._srv_res(msg, remove_resp.result.return_type, remove_resp.result.error_code)
        
        self.get_logger().info("Successfully removed all scenario objects.")    
        return self._srv_res("Successfully removed all scenario objects.", ServiceResult.SUCCESS, ServiceErrorCode.NO_ERROR)  
       
    def _call_service_with_timeout(self, client: Client, request: Any, timeout: float = 5.0) -> Any| None:    
        """
        Call a service and wait for the result with a timeout.
        Can only be used with MultiThreadedExecutor because of event.wait()
        """    
        ros_future = client.call_async(request)
        event = threading.Event()
        
        # Success callback
        ros_future.add_done_callback(lambda _: (event.set()))

        # Block until either the timer or the result sets the event
        if not event.wait(timeout=timeout):
            # Cleanup the pending request so we don't leak memory or ghost callbacks
            client.remove_pending_request(ros_future)
            self.get_logger().warning(f"Service '{client.srv_name}' timed out after {timeout}s.")
            return None
        return ros_future.result() if ros_future.done() else None
    
    def _srv_res(self, msg:str, ret_type: int, err_code: int | ServiceErrorCode) -> ServiceResult:
        result = ServiceResult()
        result.message = msg
        result.return_type = ret_type

        srv_err_code= ServiceErrorCode()
        srv_err_code.error_code = err_code
        
        result.error_code = srv_err_code
        return result

    def _no_response_srv_res(self, service: Client) -> ServiceResult:
        self.get_logger().error(f"No response from service {service.service_name}")
        return self._srv_res(f"No response from service {service.service_name}", ServiceResult.ERROR, ServiceErrorCode.ERROR_NO_RESPONSE)
        


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node = ScenarioManager()
    executor.add_node(node)
    # only make service calls after executor is spinning and node added to executor
    node.load_initial_scenario()
    try:
        spin_thread.join()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    executor.remove_node(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()