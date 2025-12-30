import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, OpaqueFunction
from launch_ros.actions import Node


def _spawn_cube(context, *args, **kwargs):
    pkg = get_package_share_directory("multi_robot_arm")

    # Use a xacro so Gazebo material extensions are included
    cube_xacro = os.path.join(pkg, "urdf", "grasp_cube.urdf.xacro")
    # If you have a plain .urdf instead, set cube_xacro to that path and skip xacro processing.

    if cube_xacro.endswith(".xacro"):
        doc = xacro.process_file(cube_xacro)
        cube_xml = doc.toprettyxml(indent="  ")
    else:
        with open(cube_xacro, "r") as f:
            cube_xml = f.read()

    out_urdf = "/tmp/grasp_cube_generated.urdf"
    with open(out_urdf, "w") as f:
        f.write(cube_xml)

    return [
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "grasp_cube",
                "-file", out_urdf,
                "-x", "0.0",
                "-y", "1.0",
                "-z", "0.05",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    # Gazebo (server+client) with ROS factory so spawn_entity works
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Give Gazebo time to come up, then generate URDF and spawn
    spawn_delayed = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=_spawn_cube)]
    )

    return LaunchDescription([gazebo, spawn_delayed])
