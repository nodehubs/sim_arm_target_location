# Copyright (c) 2022，Horizon Robotics.
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_sample_image_width", default_value=TextSubstitution(text="640")
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_sample_image_height", default_value=TextSubstitution(text="480")
    )

    web_show = os.getenv('WEB_SHOW')
    print("web_show is ", web_show)

    # jpeg图片编码&发布pkg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_decode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'ros',
            'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/camera_node/image_raw/compressed',
            'codec_pub_topic': '/hbmem_img',
            'codec_in_format': 'jpeg-compressed',
            'codec_out_format': 'nv12'
        }.items()
    )

    nv12_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hobot_codec'),
                'launch/hobot_codec_encode.launch.py')),
        launch_arguments={
            'codec_in_mode': 'shared_mem',
            'codec_out_mode': 'ros',
            'codec_sub_topic': '/hbmem_img',
            'codec_pub_topic': '/image_jpeg',
            'codec_in_format': 'nv12',
            'codec_out_format': 'jpeg'
        }.items()
    )


    # web展示pkg
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket.launch.py')),
        launch_arguments={
            'websocket_image_topic': '/image_jpeg',
            'websocket_smart_topic': '/sim_arm_target_detection'
        }.items()
    )

    # 目标定位pkg
    sim_arm_target_location_node = Node(
        package='sim_arm_target_location',
        executable='sim_arm_target_location',
        output='screen',
        parameters=[
            {"is_shared_mem_sub": True},
            {"sub_img_topic": "/hbmem_img"},
            {"config_file": "config/number_cube.json"},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    if web_show == "TRUE":
        return LaunchDescription([
            # 图片编解码&发布pkg
            jpeg_codec_node,
            nv12_codec_node,
            # 启动算法pkg
            sim_arm_target_location_node,
            web_node
        ])
    else:
        return LaunchDescription([
            jpeg_codec_node,
            sim_arm_target_location_node
        ])

