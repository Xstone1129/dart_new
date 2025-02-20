import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, Shutdown
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    node_params = os.path.join(
        get_package_share_directory('rm_vision_main'), 'config', 'node_params.yaml'
    )
    
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('rm_vision_main'), 'config', 'launch_params.yaml')))
    
    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='rm_light_detector',
                    plugin='rm_auto_light::DetectorNode',
                    name='detector_node',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                        'armor_detector:=' + launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

        # 定义检测器节点
   
    detector_node = Node(            
        package='rm_light_detector',
        executable='light_detector_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                'detect_node:=' + launch_params['detector_log_level']],
    )
    
    rm_auto_record_node = Node(
        package='rm_auto_record',
        executable='rm_auto_record_node',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                'rm_auto_record:=INFO'],
    )  

    img_push_node = None
    if not launch_params['using_camera']:
        launch_params['using_serial_port'] = False
        launch_params['using_record'] = False
        img_push_node = Node(
            package='img_test',
            executable='img_push',
        )
        
    delay_serial_node = None
    if launch_params['using_serial_port']:
        serial_driver_node = Node(
            package='rm_serial_driver',
            executable='rm_serial_driver_node',
            output='both',
            emulate_tty=True,
            parameters=[node_params],
            on_exit=Shutdown(),
            ros_arguments=['--ros-args', '--log-level',
                        'serial_driver:=' + launch_params['serial_log_level']],
        )
        # 延时启动串口驱动节点
        delay_serial_node = TimerAction(
            period=0.5,
            actions=[serial_driver_node],
        )

    delay_cam_node = None
    if launch_params['using_camera']:
        hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
        mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')
        gx_camera_node = get_camera_node('galaxy_camera', 'galaxy_camera::GalaxyCameraNode')
        # 根据配置选择相机和容器
        if launch_params['camera'] == 'hik':
            cam_detector = get_camera_detector_container(hik_camera_node)
        elif launch_params['camera'] == 'mv':
            cam_detector = get_camera_detector_container(mv_camera_node)
        elif launch_params['camera'] == 'gx':
            cam_detector = get_camera_detector_container(gx_camera_node)
        # 延时启动相机和检测容器
        delay_cam_node = TimerAction(
            period=0.5,
            actions=[cam_detector],
        )
    else:
        # 使用检测器节点
        delay_cam_node = TimerAction(
            period=0.5,
            actions=[detector_node],
        )
        
    delay_recoder_node = None
    if launch_params['using_record']:
        delay_recoder_node = TimerAction(
            period=1.5,
            actions=[rm_auto_record_node],
        )
    

    launch_description = LaunchDescription()
    if img_push_node:
        launch_description.add_action(img_push_node)
    if delay_serial_node:
        launch_description.add_action(delay_serial_node)
    if delay_cam_node:
        launch_description.add_action(delay_cam_node)
    if delay_recoder_node:
        launch_description.add_action(delay_recoder_node)    

    return launch_description
