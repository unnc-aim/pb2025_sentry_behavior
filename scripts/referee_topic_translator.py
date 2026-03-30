#!/usr/bin/env python3

# Copyright 2025 LijieZhou
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

"""
Referee Topic Translator Node

Subscribes to dji_referee_protocol's typed ROS2 msg topics and
republishes them as typed pb_rm_interfaces messages that
pb2025_sentry_behavior can consume.

This node does NOT access the serial port directly, making it resilient
to hardware issues. It relies on a running dji_referee_protocol node
(local or remote) that handles serial communication.

Input topics (from dji_referee_protocol):
    /referee/common/game_status (dji_referee_protocol/msg/GameStatus)
    /referee/common/robot_hp (dji_referee_protocol/msg/RobotHP)
    /referee/common/field_event (dji_referee_protocol/msg/FieldEvent)
    /referee/common/robot_performance (dji_referee_protocol/msg/RobotPerformance)
    /referee/common/robot_heat (dji_referee_protocol/msg/RobotHeat)
    /referee/common/robot_position (dji_referee_protocol/msg/RobotPosition)
    /referee/common/robot_buff (dji_referee_protocol/msg/RobotBuff)
    /referee/common/damage_state (dji_referee_protocol/msg/DamageState)
    /referee/common/allowed_shoot (dji_referee_protocol/msg/AllowedShoot)
    /referee/common/rfid_status (dji_referee_protocol/msg/RFIDStatus)
    /referee/common/ground_robot_position (dji_referee_protocol/msg/GroundRobotPosition)

Output topics (for behavior tree):
    referee/game_status (pb_rm_interfaces/msg/GameStatus)
    referee/all_robot_hp (pb_rm_interfaces/msg/GameRobotHP)
    referee/robot_status (pb_rm_interfaces/msg/RobotStatus)
    referee/rfid_status (pb_rm_interfaces/msg/RfidStatus)
    referee/event_data (pb_rm_interfaces/msg/EventData)
    referee/buff (pb_rm_interfaces/msg/Buff)
    referee/ground_robot_position (pb_rm_interfaces/msg/GroundRobotPosition)
"""

import math
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from pb_rm_interfaces.msg import Buff
from pb_rm_interfaces.msg import EventData
from pb_rm_interfaces.msg import GameRobotHP
from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus
from pb_rm_interfaces.msg import RobotStatus
from rclpy.node import Node

# Import dji_referee_protocol message types
from dji_referee_protocol.msg import (
    AllowedShoot as DjiAllowedShoot,
    DamageState as DjiDamageState,
    FieldEvent as DjiFieldEvent,
    GameStatus as DjiGameStatus,
    GroundRobotPosition as DjiGroundRobotPosition,
    RobotBuff as DjiRobotBuff,
    RobotHP as DjiRobotHP,
    RobotHeat as DjiRobotHeat,
    RobotPerformance as DjiRobotPerformance,
    RobotPosition as DjiRobotPosition,
    RFIDStatus as DjiRFIDStatus,
)


class RefereeTopicTranslator(Node):
    """Translate dji_referee_protocol typed topics -> pb_rm_interfaces messages."""

    def __init__(self) -> None:
        super().__init__("referee_topic_translator")

        self.declare_parameter("input_topic_prefix", "/referee/common")
        self.declare_parameter("output_topic_prefix", "referee")
        self.declare_parameter("publish_rate_hz", 10.0)

        input_prefix = str(self.get_parameter("input_topic_prefix").value)
        self.output_prefix = str(self.get_parameter("output_topic_prefix").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # Accumulated state for composite RobotStatus message
        self.robot_status_msg = RobotStatus()
        self.game_status_msg = GameStatus()
        self.all_robot_hp_msg = GameRobotHP()
        self.event_data_msg = EventData()
        self.rfid_status_msg = RfidStatus()
        self.buff_msg = Buff()
        self.ground_robot_position_msg = GroundRobotPosition()
        self.damage_latched = False

        # Receipt tracking: only publish after first real message received
        self._received_game_status = False
        self._received_robot_hp = False
        self._received_event_data = False
        self._received_robot_status = False
        self._received_rfid_status = False
        self._received_buff = False
        self._received_ground_robot_position = False

        # --- Publishers (typed, for behavior tree) ---
        self.game_status_pub = self.create_publisher(
            GameStatus, f"{self.output_prefix}/game_status", 10
        )
        self.all_robot_hp_pub = self.create_publisher(
            GameRobotHP, f"{self.output_prefix}/all_robot_hp", 10
        )
        self.robot_status_pub = self.create_publisher(
            RobotStatus, f"{self.output_prefix}/robot_status", 10
        )
        self.rfid_status_pub = self.create_publisher(
            RfidStatus, f"{self.output_prefix}/rfid_status", 10
        )
        self.event_data_pub = self.create_publisher(
            EventData, f"{self.output_prefix}/event_data", 10
        )
        self.buff_pub = self.create_publisher(
            Buff, f"{self.output_prefix}/buff", 10
        )
        self.ground_robot_position_pub = self.create_publisher(
            GroundRobotPosition, f"{self.output_prefix}/ground_robot_position", 10
        )

        # --- Subscribers (typed, from dji_referee_protocol) ---
        self._subs = []

        # GameStatus subscription
        sub = self.create_subscription(
            DjiGameStatus, f"{input_prefix}/game_status", self._on_game_status, 10
        )
        self._subs.append(sub)

        # RobotHP subscription
        sub = self.create_subscription(
            DjiRobotHP, f"{input_prefix}/robot_hp", self._on_robot_hp, 10
        )
        self._subs.append(sub)

        # FieldEvent subscription
        sub = self.create_subscription(
            DjiFieldEvent, f"{input_prefix}/field_event", self._on_field_event, 10
        )
        self._subs.append(sub)

        # RobotPerformance subscription
        sub = self.create_subscription(
            DjiRobotPerformance, f"{input_prefix}/robot_performance", self._on_robot_performance, 10
        )
        self._subs.append(sub)

        # RobotHeat subscription
        sub = self.create_subscription(
            DjiRobotHeat, f"{input_prefix}/robot_heat", self._on_robot_heat, 10
        )
        self._subs.append(sub)

        # RobotPosition subscription
        sub = self.create_subscription(
            DjiRobotPosition, f"{input_prefix}/robot_position", self._on_robot_position, 10
        )
        self._subs.append(sub)

        # RobotBuff subscription
        sub = self.create_subscription(
            DjiRobotBuff, f"{input_prefix}/robot_buff", self._on_robot_buff, 10
        )
        self._subs.append(sub)

        # DamageState subscription
        sub = self.create_subscription(
            DjiDamageState, f"{input_prefix}/damage_state", self._on_damage_state, 10
        )
        self._subs.append(sub)

        # AllowedShoot subscription
        sub = self.create_subscription(
            DjiAllowedShoot, f"{input_prefix}/allowed_shoot", self._on_allowed_shoot, 10
        )
        self._subs.append(sub)

        # RFIDStatus subscription
        sub = self.create_subscription(
            DjiRFIDStatus, f"{input_prefix}/rfid_status", self._on_rfid_status, 10
        )
        self._subs.append(sub)

        # GroundRobotPosition subscription
        sub = self.create_subscription(
            DjiGroundRobotPosition, f"{input_prefix}/ground_robot_position", self._on_ground_robot_position, 10
        )
        self._subs.append(sub)

        # Periodic publish timer
        period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.1
        self.publish_timer = self.create_timer(period, self._publish_all)

        self.get_logger().info(
            f"Referee topic translator started: "
            f"input={input_prefix}/*, output={self.output_prefix}/*"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _rfid_bit(bits_low: int, bits_high: int, index: int) -> bool:
        if index < 32:
            return bool(bits_low & (1 << index))
        return bool(bits_high & (1 << (index - 32)))

    # ------------------------------------------------------------------
    # DJI topic handlers (typed messages)
    # ------------------------------------------------------------------
    def _on_game_status(self, msg: DjiGameStatus) -> None:
        self._received_game_status = True
        self.game_status_msg.game_progress = int(msg.game_progress)
        self.game_status_msg.stage_remain_time = int(msg.stage_remain_time)

    def _on_robot_hp(self, msg: DjiRobotHP) -> None:
        self._received_robot_hp = True
        self.all_robot_hp_msg.red_1_robot_hp = int(msg.red_1_robot_hp)
        self.all_robot_hp_msg.red_2_robot_hp = int(msg.red_2_robot_hp)
        self.all_robot_hp_msg.red_3_robot_hp = int(msg.red_3_robot_hp)
        self.all_robot_hp_msg.red_4_robot_hp = int(msg.red_4_robot_hp)
        self.all_robot_hp_msg.red_7_robot_hp = int(msg.red_7_robot_hp)
        self.all_robot_hp_msg.red_outpost_hp = int(msg.red_outpost_hp)
        self.all_robot_hp_msg.red_base_hp = int(msg.red_base_hp)
        self.all_robot_hp_msg.blue_1_robot_hp = int(msg.blue_1_robot_hp)
        self.all_robot_hp_msg.blue_2_robot_hp = int(msg.blue_2_robot_hp)
        self.all_robot_hp_msg.blue_3_robot_hp = int(msg.blue_3_robot_hp)
        self.all_robot_hp_msg.blue_4_robot_hp = int(msg.blue_4_robot_hp)
        self.all_robot_hp_msg.blue_7_robot_hp = int(msg.blue_7_robot_hp)
        self.all_robot_hp_msg.blue_outpost_hp = int(msg.blue_outpost_hp)
        self.all_robot_hp_msg.blue_base_hp = int(msg.blue_base_hp)

    def _on_field_event(self, msg: DjiFieldEvent) -> None:
        self._received_event_data = True
        self.event_data_msg.non_overlapping_supply_zone = (
            EventData.OCCUPIED_FRIEND if msg.supply_area_1 else EventData.UNOCCUPIED
        )
        self.event_data_msg.overlapping_supply_zone = (
            EventData.OCCUPIED_FRIEND if msg.supply_area_2 else EventData.UNOCCUPIED
        )
        self.event_data_msg.supply_zone = (
            EventData.OCCUPIED_FRIEND if msg.rmul_supply_area else EventData.UNOCCUPIED
        )
        self.event_data_msg.small_energy = int(msg.small_energy_mech)
        self.event_data_msg.big_energy = int(msg.big_energy_mech)
        self.event_data_msg.central_highland = int(msg.central_highland)
        self.event_data_msg.trapezoidal_highland = (
            EventData.OCCUPIED_FRIEND if msg.trapezoid_highland else EventData.UNOCCUPIED
        )
        # Note: center_buff_point maps to center_gain_zone in pb_rm_interfaces
        # If needed, add mapping here

    def _on_robot_performance(self, msg: DjiRobotPerformance) -> None:
        self._received_robot_status = True
        self.robot_status_msg.robot_id = int(msg.robot_id)
        self.robot_status_msg.robot_level = int(msg.robot_level)
        self.robot_status_msg.current_hp = int(msg.current_hp)
        self.robot_status_msg.maximum_hp = int(msg.maximum_hp)
        self.robot_status_msg.shooter_barrel_cooling_value = int(msg.shooter_barrel_cooling_value)
        self.robot_status_msg.shooter_barrel_heat_limit = int(msg.shooter_barrel_heat_limit)

    def _on_robot_heat(self, msg: DjiRobotHeat) -> None:
        self._received_robot_status = True
        self.robot_status_msg.shooter_17mm_1_barrel_heat = int(msg.shooter_17mm_barrel_heat)

    def _on_robot_position(self, msg: DjiRobotPosition) -> None:
        self._received_robot_status = True
        yaw = math.radians(float(msg.angle))
        self.robot_status_msg.robot_pos = Pose(
            position=Point(x=float(msg.x), y=float(msg.y), z=0.0),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )

    def _on_robot_buff(self, msg: DjiRobotBuff) -> None:
        self._received_buff = True
        self.buff_msg.recovery_buff = int(msg.recovery_buff)
        self.buff_msg.cooling_buff = int(msg.cooling_buff)
        self.buff_msg.defence_buff = int(msg.defence_buff)
        self.buff_msg.vulnerability_buff = int(msg.vulnerability_buff)
        self.buff_msg.attack_buff = int(msg.attack_buff)
        self.buff_msg.remaining_energy = int(msg.remaining_energy)

    def _on_damage_state(self, msg: DjiDamageState) -> None:
        self._received_robot_status = True
        self.robot_status_msg.armor_id = int(msg.armor_id)
        self.robot_status_msg.hp_deduction_reason = int(msg.damage_type)
        self.robot_status_msg.is_hp_deduced = True
        self.damage_latched = True

    def _on_allowed_shoot(self, msg: DjiAllowedShoot) -> None:
        self._received_robot_status = True
        self.robot_status_msg.projectile_allowance_17mm = int(msg.projectile_allowance_17mm)
        self.robot_status_msg.remaining_gold_coin = int(msg.remaining_gold_coin)

    def _on_rfid_status(self, msg: DjiRFIDStatus) -> None:
        self._received_rfid_status = True
        bits_low = int(msg.detected_rfid_bits_low)
        bits_high = int(msg.detected_rfid_bits_high)
        b = self._rfid_bit
        self.rfid_status_msg.base_gain_point = b(bits_low, bits_high, 0)
        self.rfid_status_msg.central_highland_gain_point = b(bits_low, bits_high, 1)
        self.rfid_status_msg.enemy_central_highland_gain_point = b(bits_low, bits_high, 2)
        self.rfid_status_msg.friendly_trapezoidal_highland_gain_point = b(bits_low, bits_high, 3)
        self.rfid_status_msg.enemy_trapezoidal_highland_gain_point = b(bits_low, bits_high, 4)
        self.rfid_status_msg.friendly_fly_ramp_front_gain_point = b(bits_low, bits_high, 5)
        self.rfid_status_msg.friendly_fly_ramp_back_gain_point = b(bits_low, bits_high, 6)
        self.rfid_status_msg.enemy_fly_ramp_front_gain_point = b(bits_low, bits_high, 7)
        self.rfid_status_msg.enemy_fly_ramp_back_gain_point = b(bits_low, bits_high, 8)
        self.rfid_status_msg.friendly_central_highland_lower_gain_point = b(bits_low, bits_high, 9)
        self.rfid_status_msg.friendly_central_highland_upper_gain_point = b(bits_low, bits_high, 10)
        self.rfid_status_msg.enemy_central_highland_lower_gain_point = b(bits_low, bits_high, 11)
        self.rfid_status_msg.enemy_central_highland_upper_gain_point = b(bits_low, bits_high, 12)
        self.rfid_status_msg.friendly_highway_lower_gain_point = b(bits_low, bits_high, 13)
        self.rfid_status_msg.friendly_highway_upper_gain_point = b(bits_low, bits_high, 14)
        self.rfid_status_msg.enemy_highway_lower_gain_point = b(bits_low, bits_high, 15)
        self.rfid_status_msg.enemy_highway_upper_gain_point = b(bits_low, bits_high, 16)
        self.rfid_status_msg.friendly_fortress_gain_point = b(bits_low, bits_high, 17)
        self.rfid_status_msg.friendly_outpost_gain_point = b(bits_low, bits_high, 18)
        self.rfid_status_msg.friendly_supply_zone_non_exchange = b(bits_low, bits_high, 19)
        self.rfid_status_msg.friendly_supply_zone_exchange = b(bits_low, bits_high, 20)
        self.rfid_status_msg.friendly_big_resource_island = b(bits_low, bits_high, 21)
        self.rfid_status_msg.enemy_big_resource_island = b(bits_low, bits_high, 22)
        self.rfid_status_msg.center_gain_point = b(bits_low, bits_high, 23)

    def _on_ground_robot_position(self, msg: DjiGroundRobotPosition) -> None:
        self._received_ground_robot_position = True
        self.ground_robot_position_msg.hero_position = Point(
            x=float(msg.hero_x), y=float(msg.hero_y), z=0.0
        )
        self.ground_robot_position_msg.engineer_position = Point(
            x=float(msg.engineer_x), y=float(msg.engineer_y), z=0.0
        )
        self.ground_robot_position_msg.standard_3_position = Point(
            x=float(msg.infantry_3_x), y=float(msg.infantry_3_y), z=0.0
        )
        self.ground_robot_position_msg.standard_4_position = Point(
            x=float(msg.infantry_4_x), y=float(msg.infantry_4_y), z=0.0
        )

    # ------------------------------------------------------------------
    # Periodic publish
    # ------------------------------------------------------------------
    def _publish_all(self) -> None:
        if self._received_game_status:
            self.game_status_pub.publish(self.game_status_msg)
        if self._received_robot_hp:
            self.all_robot_hp_pub.publish(self.all_robot_hp_msg)
        if self._received_robot_status:
            self.robot_status_pub.publish(self.robot_status_msg)
        if self._received_rfid_status:
            self.rfid_status_pub.publish(self.rfid_status_msg)
        if self._received_event_data:
            self.event_data_pub.publish(self.event_data_msg)
        if self._received_buff:
            self.buff_pub.publish(self.buff_msg)
        if self._received_ground_robot_position:
            self.ground_robot_position_pub.publish(self.ground_robot_position_msg)
        if self.damage_latched:
            self.robot_status_msg.is_hp_deduced = False
            self.damage_latched = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RefereeTopicTranslator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        logger = rclpy.logging.get_logger("referee_topic_translator_main")
        logger.fatal(f"Fatal error: {ex}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
