# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from __future__ import annotations

import math
import threading
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose


class RobotAPI:
    def __init__(self, config_yaml):
        self._map_frame: str = (
            config_yaml.get('map_frame')
            if isinstance(config_yaml, dict) else None
        ) or 'map'
        self._amcl_topic: str = (
            config_yaml.get('amcl_pose_topic')
            if isinstance(config_yaml, dict) else None
        ) or '/amcl_pose'
        self._nav_action_name: str = (
            config_yaml.get('nav2_action_name')
            if isinstance(config_yaml, dict) else None
        ) or '/navigate_to_pose'
        self.timeout = 5.0
        self.debug = bool(config_yaml.get('debug', False)) if isinstance(config_yaml, dict) else False

        # Internal state
        self._node: Optional[Node] = None
        self._nav_client: Optional[ActionClient] = None
        self._last_goal_handle = None
        self._last_pose: Optional[List[float]] = None  # [x, y, yaw]
        self._last_cmd_done: bool = True

        # Start a lightweight ROS 2 node dedicated to this API
        if not rclpy.ok():
            rclpy.init()
        self._node = Node('fleet_adapter_nav2_client')

        # Action client for Nav2
        self._nav_client = ActionClient(self._node, NavigateToPose, self._nav_action_name)

        self._node.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_topic,
            self._amcl_cb,
            10,
        )
        self._cmd_vel_pub = self._node.create_publisher(Twist, 'cmd_vel', 10)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] spin error: {e}')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._last_pose = [p.x, p.y, yaw]

    def check_connection(self):
        ''' Return True if we can reach the Nav2 action server. '''
        if self._nav_client is None:
            return False
        return self._nav_client.wait_for_server(timeout_sec=0.5)

    def navigate(
        self,
        robot_name: str,
        pose,
        map_name: str,
        speed_limit=0.0
    ):
        try:
            if self._nav_client is None:
                return False
            if not self._nav_client.wait_for_server(timeout_sec=2.0):
                if self.debug:
                    print('[RobotAPI] Nav2 action server not available')
                return False

            goal = NavigateToPose.Goal()
            frame = 'map'
            goal.pose.header.frame_id = frame
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()

            # Fill pose
            x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.position.z = 0.0
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            # Send goal
            send_future = self._nav_client.send_goal_async(goal)

            self._last_cmd_done = False

            def _goal_response_cb(fut):
                goal_handle = fut.result()
                self._last_goal_handle = goal_handle
                if not goal_handle.accepted:
                    if self.debug:
                        print('[RobotAPI] Nav goal rejected')
                    self._last_cmd_done = True
                    return
                result_future = goal_handle.get_result_async()

                def _result_cb(rfut):
                    # Regardless of success/abortion, mark completion so RMF can proceed
                    self._last_cmd_done = True
                result_future.add_done_callback(_result_cb)

            send_future.add_done_callback(_goal_response_cb)

            if self.debug:
                print(f'[RobotAPI] Sent NavigateToPose to ({x:.2f}, {y:.2f}, {yaw:.2f}) frame={frame}')

            return True
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] navigate error: {e}')
            return False

    def start_activity(
        self,
        robot_name: str,
        activity: str,
        label: str
    ):
        return True

    def stop(self, robot_name: str):
        ''' Cancel the current Nav2 goal and send zero cmd_vel. '''
        try:
            if self._last_goal_handle is not None:
                self._last_goal_handle.cancel_goal_async()
            # publish zero twist once
            self._cmd_vel_pub.publish(Twist())
            self._last_cmd_done = True
            return True
        except Exception as e:
            if self.debug:
                print(f'[RobotAPI] stop error: {e}')
            return False

    def position(self, robot_name: str):
        ''' Return [x, y, yaw] from the latest AMCL estimate (or None). '''
        return self._last_pose

    def battery_soc(self, robot_name: str):
        ''' Placeholder SOC (no battery wiring here). '''
        return 1.0

    def map(self, robot_name: str):
        ''' Return the current map frame name used for Nav2 goals. '''
        return "L1"

    def is_command_completed(self):
        ''' True when the latest navigation/action request has finished. '''
        return self._last_cmd_done

    def get_data(self, robot_name: str):
        ''' Returns a RobotUpdateData for one robot if a name is given. Otherwise
        return a list of RobotUpdateData for all robots. '''
        map = self.map(robot_name)
        position = self.position(robot_name)
        battery_soc = self.battery_soc(robot_name)
        if not (map is None or position is None or battery_soc is None):
            return RobotUpdateData(robot_name, map, position, battery_soc)
        return None


class RobotUpdateData:
    ''' Update data for a single robot. '''
    def __init__(self,
                 robot_name: str,
                 map: str,
                 position: list[float],
                 battery_soc: float,
                 requires_replan: bool | None = None):
        self.robot_name = robot_name
        self.position = position
        self.map = map
        self.battery_soc = battery_soc
        self.requires_replan = requires_replan
