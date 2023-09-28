import sys
from launch import LaunchDescription
import launch_ros.actions
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
import copy
from os import path
import yaml
from launch_ros.actions import Node
import os
import time
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction

params_file = get_package_share_directory("astra_camera") + "/params/dabai_dcw_params.yaml"
if not path.exists(params_file):
    print("path %s is not exists" % params_file)
    sys.exit(-1)
with open(params_file, 'r') as file:
    default_params = yaml.safe_load(file)

def duplicate_params(general_params, posix, serial_number):
    local_params = copy.deepcopy(general_params)
    local_params["camera_name"] += posix
    local_params["serial_number"] = serial_number
    return local_params

# MK4
# serial_number1 = "CH2R73100JN" #front-up CH2R73100ES
# serial_number2 = "CH282310048" #front-down
# serial_number3 = "CH2B53100KR" #back

# MK5
serial_number1 = "CH2B531000D" #front-up CH2R73100ES
serial_number2 = "CH2B531001V" #front-down
serial_number3 = "CH2B531001S" #back

params1 = duplicate_params(default_params, "1", serial_number1)
params2 = duplicate_params(default_params, "2", serial_number2)
params3 = duplicate_params(default_params, "3", serial_number3)

def func(context, *args, **kwargs):
    camera_name = kwargs['camera_name']
    params = kwargs['parameters']
    print('*********camera_name: ', params['camera_name'] )
    comps =[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='camera',
                namespace=camera_name,
                parameters=[params]
            ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                namespace=camera_name,
                name='point_cloud_xyz'),
            ]

    time.sleep(3)
    target_container_name = camera_name + '/astra_camera_container'
    return [LoadComposableNodes(composable_node_descriptions=comps, target_container=target_container_name)]

camera_1 = os.environ.get('CAMERA1_NUMBER')
camera_2 = os.environ.get('CAMERA2_NUMBER')
camera_3 = os.environ.get('CAMERA3_NUMBER')

def generate_container_node(camera_name, params):
    return ComposableNodeContainer(
        name='astra_camera_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(package='astra_camera',
            #                plugin='astra_camera::OBCameraNodeFactory',
            #                name='camera',
            #                parameters=[params],
            #                namespace=camera_name),
            # ComposableNode(package='astra_camera',
            #                plugin='astra_camera::PointCloudXyzNode',
            #                namespace=camera_name,
            #                name='point_cloud_xyz'),
            # ComposableNode(package='astra_camera',
            #                plugin='astra_camera::PointCloudXyzrgbNode',
            #                namespace=camera_name,
            #                name='point_cloud_xyzrgb')
        ],
        output='screen',
        respawn=True)


def duplicate_params(general_params, posix, serial_number):
    local_params = copy.deepcopy(general_params)
    local_params["camera_name"] += posix
    local_params["serial_number"] = serial_number
    return local_params


def generate_launch_description():
    params_file = get_package_share_directory("astra_camera") + "/params/dabai_dcw_params.yaml"
    if not path.exists(params_file):
        print("path %s is not exists" % params_file)
        sys.exit(-1)
    with open(params_file, 'r') as file:
        default_params = yaml.safe_load(file)

    container1 = generate_container_node("camera1", params1)
    container2 = generate_container_node("camera2", params2)
    container3 = generate_container_node("camera3", params3)
    
    # dummy static transformation from camera1 to camera2
    # dummy_tf_node = launch_ros.actions.Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "0",
    #         "1",
    #         "1",
    #         "0",
    #         "0",
    #         "0",
    #         "camera1_link",
    #         "camera2_link",
    #     ],
    # )

    # dummy_tf_node1 = launch_ros.actions.Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "0",
    #         "camera1_link",
    #         "camera3_link",
    #     ],
    # )
    container1_ = ComposableNodeContainer(
        name='astra_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::OBCameraNodeFactory',
                name='camera',
                namespace='camera',
                parameters=[default_params]
                ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                namespace='camera',
                name='point_cloud_xyz'),
        ],
        output='screen',
        respawn=True
    )

    event1 = RegisterEventHandler(
        OnProcessStart(
			target_action=container1,
			on_start=[LogInfo(msg='Container start'),OpaqueFunction(function=func, kwargs={'camera_name': 'camera1', 'parameters': params1})]
		))
    event2 = RegisterEventHandler(
        OnProcessStart(
			target_action=container2,
			on_start=[LogInfo(msg='Container start'),OpaqueFunction(function=func, kwargs={'camera_name': 'camera2', 'parameters': params2})]
		))
    event3 = RegisterEventHandler(
        OnProcessStart(
			target_action=container3,
			on_start=[LogInfo(msg='Container start'),OpaqueFunction(function=func, kwargs={'camera_name': 'camera3', 'parameters': params3})]
		))

    containers = [
        event1,container1,
        event2,container2,
        event3,container3,
    ]
    return LaunchDescription(containers)
