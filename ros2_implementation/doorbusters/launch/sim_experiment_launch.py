#!/usr/bin/env python3

import json
import os
import launch
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

def launch_setup(context):
    launch_list = []

    num_ugvs = int(perform_substitutions(context,
                                          [LaunchConfiguration("num_ugvs")]))
    num_uavs = int(perform_substitutions(context,
                                          [LaunchConfiguration("num_uavs")]))

    ugvs = ["warty", "wilbur", "wanda", "lester", "willow"]
    uavs = ["alfa", "bravo", "charlie", "delta", "echo"]

    ugv_names = ugvs[:num_ugvs]
    uav_names = uavs[:num_uavs]
    print(f"Robot names: {ugv_names + uav_names}")

    yaml_file = LaunchConfiguration("yaml_file")
    yaml_file_arg = DeclareLaunchArgument(
        name="yaml_file",
        default_value="swarm_config.yaml",
        description=(
            "YAML file containing the experiment parameters"
        ),
    )

    # run sim
    simulator_cmd = [
        "/sim/arl-warthog-sim/Build/sim.x86_64",
        "--set-ugv-robot-count",
        str(num_ugvs),
        "--set-uav-robot-count",
        str(num_uavs),
        "utm",
        "true",
    ]

    simulator = ExecuteProcess(
        cmd=simulator_cmd,
        output="screen",
        shell=True,
    )
    launch_list.append(simulator)

    launch_list.append(Node(
        package="doorbusters",
        executable="target_publisher",
        name="global_target_publisher",
        output="screen",
    ))

    num_robots = num_ugvs + num_uavs

    for i in range(num_ugvs):
        launch_list.append(Node(
            package="doorbusters",
            executable="final_control",
            name="final_control",
            namespace=ugv_names[i],
            parameters=[{
                "is_informed": True,
                "neighborhood_id": str(i),
                "num_agents": num_robots,
            }],
            remappings=[
                (f"/{ugv_names[i]}/odom", f"/{ugv_names[i]}/platform/odom"),
            ],
            output="screen",
        ))

    for i in range(num_uavs):
        launch_list.append(Node(
            package="doorbusters",
            executable="final_control",
            name="final_control",
            namespace=uav_names[i],
            parameters=[{
                "is_informed": True,
                "neighborhood_id": str(num_ugvs + i),
                "num_agents": num_robots,
            }],
            remappings=[
                (f"/{uav_names[i]}/odom", f"/{uav_names[i]}/raw_odom"),
            ],
            output="screen",
        ))
    return launch_list

def generate_launch_description():
    ld = launch.LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
    return ld
