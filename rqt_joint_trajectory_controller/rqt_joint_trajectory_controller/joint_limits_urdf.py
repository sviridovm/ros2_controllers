#!/usr/bin/env python

# Copyright 2022 PAL Robotics S.L.
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

# Code inspired on the joint_state_publisher package by David Lu!!!
# https://github.com/ros/robot_model/blob/indigo-devel/
# joint_state_publisher/joint_state_publisher/joint_state_publisher

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

from math import pi

import rclpy
from std_msgs.msg import String

from urdf_parser_py.urdf import URDF

description = ""


def callback(msg):
    global description
    description = msg.data


def subscribe_to_robot_description(node, key="robot_description"):
    qos_profile = rclpy.qos.QoSProfile(depth=1)
    qos_profile.durability = rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    qos_profile.reliability = rclpy.qos.ReliabilityPolicy.RELIABLE

    node.create_subscription(String, key, callback, qos_profile)


def get_joint_limits(node, joints_names, use_smallest_joint_limits=True):
    use_small = use_smallest_joint_limits
    use_mimic = True

    count = 0
    while description == "" and count < 10:
        print("Waiting for the robot_description!")
        count += 1
        rclpy.spin_once(node, timeout_sec=1.0)

    if description == "":
        return {}

    # will raise exception if URDF is invalid
    robot = URDF.from_xml_string(description)

    free_joints = {}
    # dependent_joints = {}
    dependent_joints = set()

    for joint_name, joint in robot.joints.items():
        if joint.type == "fixed":
            continue

        if joint.limit is None:
            if joint in joints_names:
                # ? Is there a more specific exception we can raise here?
                raise Exception(
                    f"Missing limits tag for the joint : {joint_name} in the robot_description!"
                )
            else:
                continue

        # joint limits have default values of 0
        minval = joint.limit.lower
        maxval = joint.limit.upper

        has_position_limits = joint.type != "continuous"
        maxvel = joint.limit.velocity

        if joint.type == "continuous":
            minval = -pi
            maxval = pi

        if joint.safety_controller is not None and use_small:
            safety = joint.safety_controller
            if safety.soft_lower_limit is not None:
                minval = max(minval, safety.soft_lower_limit)
            if safety.soft_upper_limit is not None:
                maxval = min(maxval, safety.soft_upper_limit)

        if joint.mimic is not None and use_mimic:
            # ? Why do we use a map if we only check for membership by name?
            # mimic = joint.mimic
            # entry = {"parent": joint.mimic.joint}
            # if mimic.multiplier is not None:
            #     entry["factor"] = joint.mimic.multiplier
            # if mimic.offset is not None:
            #     entry["offset"] = joint.mimic.offset

            # dependent_joints[joint_name] = entry

            dependent_joints.add(joint_name)
            continue

        if joint_name in dependent_joints:
            continue

        joint_dict = {
            "min_position": minval,
            "max_position": maxval,
            "has_position_limits": has_position_limits,
            "max_velocity": maxvel,
        }
        free_joints[joint_name] = joint_dict

    return free_joints
