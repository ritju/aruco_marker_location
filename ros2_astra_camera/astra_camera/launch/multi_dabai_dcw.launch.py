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

serial_number1 = "CH2R73100ES" #"CH282310048"
serial_number2 = "CH2B531001V"
serial_number3 = "CH2B53100KR"
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

def generate_container_node(camera_name, params):
    return ComposableNodeContainer(
        name='astra_camera_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::OBCameraNodeFactory',
                           name='camera',
                           parameters=[params],
                           namespace=camera_name),
            ComposableNode(package='astra_camera',
                           plugin='astra_camera::PointCloudXyzNode',
                           namespace=camera_name,
                           name='point_cloud_xyz'),
            # ComposableNode(package='astra_camera',
            #                plugin='astra_camera::PointCloudXyzrgbNode',
            #                namespace=camera_name,
            #                name='point_cloud_xyzrgb')
        ],
        output='screen',
        respawn=True)

def generate_launch_description():   
    
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

    containers = [
        event1,container1,
        # container2,
        # container3,
    ]
    return LaunchDescription(containers)
