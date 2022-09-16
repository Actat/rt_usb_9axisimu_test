import os
import launch

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import lifecycle_msgs.msg


def generate_launch_description():
    rviz_config_name = 'default.rviz'
    rviz_path = os.path.join(
        get_package_share_directory('rt_usb_9axisimu_test'),
        'rviz',
        rviz_config_name
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path]
    )

    rqt_gui_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
    )

    imu_node = LifecycleNode(
        package='rt_usb_9axisimu_driver',
        executable='rt_usb_9axisimu_driver',
        name='rt_usb_9axisimu',
        parameters=[{
            'port': '/dev/ttyACM0'
        }]
    )
    imu_inactive_to_active = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(
                            imu_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )]
        )
    )
    imu_configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(imu_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    imu_complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_complementary_filter',
        parameters=[{
            'use_mag': True,
            'publish_tf': True,
        }]
    )

    tf_static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='broadcaster',
        arguments=['0', '0', '0', '0', '3.14', '0', 'base_link', 'odom'],
    )

    ld = LaunchDescription()
    ld.add_action(imu_inactive_to_active)
    ld.add_action(imu_node)
    ld.add_action(imu_configure)
    ld.add_action(imu_complementary_filter_node)
    ld.add_action(tf_static_transform_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(rqt_gui_node)

    return ld
