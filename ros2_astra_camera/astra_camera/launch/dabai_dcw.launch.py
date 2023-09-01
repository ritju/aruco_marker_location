import launch
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction

from ament_index_python import get_package_share_directory
import yaml
import time


params_file = get_package_share_directory("astra_camera") + "/params/dabai_dcw_params.yaml"
with open(params_file, 'r') as file:
    config_params = yaml.safe_load(file)

def func(context, *args, **kwargs):
	comps =[
                ComposableNode(
                    package='astra_camera',
                    plugin='astra_camera::OBCameraNodeFactory',
                    name='camera',
                    namespace='camera',
                    parameters=[config_params]
                ),
                ComposableNode(
                    package='astra_camera',
                    plugin='astra_camera::PointCloudXyzNode',
                    namespace='camera',
                    name='point_cloud_xyz'),
                ComposableNode(
                    package='astra_camera',
                    plugin='astra_camera::PointCloudXyzrgbNode',
                    namespace='camera',
                    name='point_cloud_xyzrgb')
            ]
	time.sleep(15) 
	return [LoadComposableNodes(composable_node_descriptions=comps, target_container='astra_camera_container')]

def generate_launch_description():
    
    container = ComposableNodeContainer(
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
                parameters=[config_params]
                ),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzNode',
                namespace='camera',
                name='point_cloud_xyz'),
            ComposableNode(
                package='astra_camera',
                plugin='astra_camera::PointCloudXyzrgbNode',
                namespace='camera',
                name='point_cloud_xyzrgb')
        ],
        output='screen',
        respawn=True
    )

    event = RegisterEventHandler(
		OnProcessStart(
			target_action=container,
			on_start=[LogInfo(msg='Container start'),OpaqueFunction(function=func)]
		))


    return launch.LaunchDescription([event, container])






















            
