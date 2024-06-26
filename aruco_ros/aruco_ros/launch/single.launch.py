from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
import yaml
import rclpy


def launch_setup(context, *args, **kwargs):

    eye = perform_substitutions(context, [LaunchConfiguration('eye')])
    marker_id_and_bluetooth_mac = ['']
    try:
        if 'marker_id_and_bluetooth_mac' in os.environ:
            marker_id_and_bluetooth_mac = os.environ.get('marker_id_and_bluetooth_mac').split(',')
            if marker_id_and_bluetooth_mac == ['']:
                raise
    except:
        print("Please input aruco marker_id and bluetooth_mac !")


    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        # 'camera_frame': 'stereo_gazebo_' + eye + '_camera_optical_frame',
        'camera_frame': 'camera3_depth_optical_frame',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'P_uncertain': LaunchConfiguration('P_uncertain'),
        'R_uncertain': LaunchConfiguration('R_uncertain'),
        'marker_id_and_bluetooth_mac': marker_id_and_bluetooth_mac,
    }

    # aruco_ros_pkg_path = get_package_share_directory('aruco_ros')
    # params_file_path = os.path.join(aruco_ros_pkg_path, 'params', 'config.yaml')
    # print(params_file_path)
    # with open(params_file_path, 'r') as file:
    #     config_params = yaml.safe_load(file)

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        # remappings=[('/camera_info', '/stereo/' + eye + '/camera_info'),
        #             ('/image', '/stereo/' + eye + '/image_rect_color')],
        remappings=[('/camera_info', '/camera3/' + 'color' + '/camera_info'),
                    ('/image', '/camera3/' + 'color' + '/image_raw')],
    )

    return [aruco_single]


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='582',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.20',
        description='Marker size in m. '
    )

    eye_arg = DeclareLaunchArgument(
        'eye', default_value='left',
        description='Eye. ',
        choices=['left', 'right'],
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    R_uncertain_arg = DeclareLaunchArgument(
        'R_uncertain', default_value='50.0',
        description='R_uncertain',
    )
    Q_uncertain_arg = DeclareLaunchArgument(
        'Q_uncertain', default_value='0.00001',
        description='Q_uncertain',
    )

    P_uncertain_arg = DeclareLaunchArgument(
        'P_uncertain', default_value='1.0',
        description='P_uncertain',
    )

    r_uncertain_x = DeclareLaunchArgument(
        'r_uncertain_x', default_value='10',
        description='r_uncertain_x',
    )

    r_uncertain_y = DeclareLaunchArgument(
        'r_uncertain_y', default_value='0.2',
        description='r_uncertain_y',
    )

    r_uncertain_theta = DeclareLaunchArgument(
        'r_uncertain_theta', default_value='0.2',
        description='r_uncertain_theta',
    )

    q_uncertain_x = DeclareLaunchArgument(
        'q_uncertain_x', default_value='0.1',
        description='q_uncertain_x',
    )

    q_uncertain_y = DeclareLaunchArgument(
        'q_uncertain_y', default_value='0.1',
        description='q_uncertain_y',
    )

    q_uncertain_theta = DeclareLaunchArgument(
        'q_uncertain_theta', default_value='0.1',
        description='q_uncertain_theta',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(eye_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)
    ld.add_action(P_uncertain_arg)
    ld.add_action(R_uncertain_arg)
    ld.add_action(Q_uncertain_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
