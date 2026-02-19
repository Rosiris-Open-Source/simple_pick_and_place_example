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

import rclpy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
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
        self.pass_through_cbg = ReentrantCallbackGroup()
        self._scenario : ScenarioInstance| None = None

        self._declare_and_get_parameters()
        if not self._initialize_services_clients(self.wait_for_services_timeout):
            self.get_logger().error("Timeout while waiting for service")
            exit(-1)

        self._initialize_service_servers()
        # load a initial scenario it one is passed via params
        if self._initial_scenario_path:
            resp = self._load_scenario(self._initial_scenario_path)
            if resp.result.return_type == ServiceResult.SUCCESS and  self._scenario is not None:
                self.get_logger().info(f"Successfully loaded scenario: {self._scenario.name}")
            else:
                self.get_logger().warning(f"Could not load initial scenario at: {self._initial_scenario_path}")
                self.get_logger().warning(f"Got response: {resp.result.return_type}, with {resp.result.error_code}.")
                self.get_logger().warning(f"Error Message: {resp.result.message}")


    def _declare_and_get_parameters(self):
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
        path_to_scenario_descr = ParameterDescriptor(
            description='Path to the scenario which should be loaded on startup.',
            type=Parameter.Type.STRING,
        )
        # Only None is defined as wait forever in wait_for_service...
        self._initial_scenario_path: str = self.declare_parameter('path_to_scenario', "", path_to_scenario_descr).value
        self.get_logger().info(f"initial_scenario_path = {self._initial_scenario_path}")


    def _initialize_services_clients(self, service_wait_timeout: int| None = None) -> bool:
        self.add_cli = self.create_client(
            AddCollisionObjects, "/planning_scene_manager/add_collision_objects", callback_group=self.pass_through_cbg)
        self.move_cli = self.create_client(
            MoveCollisionObjects, "/planning_scene_manager/move_collision_objects", callback_group=self.pass_through_cbg)
        self.attach_cli = self.create_client(
            AttachCollisionObject, "/planning_scene_manager/attach_collision_object", callback_group=self.pass_through_cbg)
        self.detach_cli = self.create_client(
            DetachCollisionObject, "/planning_scene_manager/detach_collision_object", callback_group=self.pass_through_cbg)
        self.remove_cli = self.create_client(
            RemoveCollisionObjects, "/planning_scene_manager/remove_collision_objects", callback_group=self.pass_through_cbg)
        self.acm_cli = self.create_client(
            UpdateAllowedCollisions, "/planning_scene_manager/update_allowed_collisions", callback_group=self.pass_through_cbg)
        
        for cli in [
            self.add_cli, self.move_cli, self.attach_cli,
            self.detach_cli, self.remove_cli, self.acm_cli
        ]:
            self.get_logger().info(f"Waiting for {cli.service_name} to become ready.")
            if not cli.wait_for_service(service_wait_timeout):
                self.get_logger().warning(f"Timeout reached while waiting for {cli.service_name}")
                return False
            self.get_logger().info(f"{cli.service_name} ready.")
            
        return True
                
    def _initialize_service_servers(self):
        self.setup_scenario_srv = self.create_service(LoadScenario, '~/load_scenario', self._load_scenario_cb, callback_group=self.pass_through_cbg)
        self.reset_scenario_srv = self.create_service(ResetScenario, '~/reset_scenario', self._reset_scenario_cb, callback_group=self.pass_through_cbg)


    def _load_scenario_cb(self, req: LoadScenario.Request, resp: LoadScenario.Response):
        scenario_path = req.path_to_scenario_file
        if not scenario_path or not scenario_path.strip():
            resp.result = self._srv_res(f"No path provided.", ServiceResult.ERROR, ServiceErrorCode.ERROR_VALUE)
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


    def _reset_scenario_cb(self, _: ResetScenario.Request, resp: ResetScenario.Response):
        err_msg = "Resetting of scenario not implemented"
        if self._scenario is None:
            err_msg = "No scenario loaded, cannot reset."
            self.get_logger().warning(err_msg)
            resp.success = False
            resp.message = err_msg
            return resp
        resp.result = self._load_scenario_into_scene(self._scenario)
        return resp

    def _call_and_wait(self, client: Client, request) -> Any | None:
        future : Future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future=future)
        return future.result()
    
    def _srv_res(self, msg:str, ret_type: int, err_code: int | ServiceErrorCode) -> ServiceResult:
        result = ServiceResult()
        result.message = msg
        result.return_type = ret_type

        srv_err_code= ServiceErrorCode()
        srv_err_code.error_code = err_code
        
        result.error_code = srv_err_code
        return result
    
    def _load_scenario_into_scene(self, scenario: ScenarioInstance) -> ServiceResult:
        res = ServiceResult()
        self.get_logger().info(f"Loading scenario into scene: {scenario.name}")
        objs_to_add, col_mtrx_upds = scenario.to_msg()
        # add collision obj from scenario:
        add_obj_req = AddCollisionObjects.Request()
        add_obj_req.collision_objects = objs_to_add
        add_cli_resp : AddCollisionObjects.Response | None = self._call_and_wait(self.add_cli, add_obj_req)
        
        if add_cli_resp is None:
            res = self._srv_res(f"No response from service {self.add_cli}", ServiceResult.ERROR, ServiceErrorCode.ERROR)
            return res
        if add_cli_resp.result.return_type != ServiceResult.SUCCESS:
            res = self._srv_res(f"Failed to load scenario: {add_cli_resp.result.message}", add_cli_resp.result.return_type, add_cli_resp.result.error_code)
            return res

        scenario.loaded_in_scene = True
        success_msg = f"Successful setup up the {scenario.name} scenario."
        self.get_logger().info(success_msg)
        res = self._srv_res(success_msg, ServiceResult.SUCCESS, ServiceErrorCode.NO_ERROR)
        return res
    
def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = ScenarioManager()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()