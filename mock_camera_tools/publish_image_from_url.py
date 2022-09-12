# Copyright 2022 Jacob Perron
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

import argparse
import sys
import threading
from typing import Optional
from typing import Text

from PIL import Image
import requests
from io import BytesIO

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import CameraInfo


def parse_arguments(args):
    parser = argparse.ArgumentParser(
        description='Load an image from the provided URL and publish it on a ROS topic'
    )
    parser.add_argument('url', type=str, help='URL of the image to load.')
    parser.add_argument(
        '--show', '-s', action='store_true', help='Show the image that was loaded.'
    )
    parser.add_argument(
        '--camera-info', '-c', action='store_true',
        help='Publish sensor_msgs/msg/CameraInfo messages.'
    )
    parser.add_argument(
        '--frame-id', '-f', type=str, default='base_link',
        help='Set the the frame_id member of the published message to this value.'
    )
    parser.add_argument(
        '--period', '-p', type=float, default=1.0,
        help='Publish messages at this period (every N seconds).'
    )
    camera_info_group = parser.add_argument_group('Camera info options')
    camera_info_group.add_argument(
        '--fx', type=float, default=1.0,
        help='X-focal length for populating the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--fy', type=float, default=1.0,
        help='Y-focal length for populating the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--cx', type=float, default=None,
        help='Camera center X-coordinate for populating the intrinsic matrix. '
             'Defaults to half the image width.'
    )
    camera_info_group.add_argument(
        '--cy', type=float, default=None,
        help='Camera center Y-coordinate for populating the intrinsic matrix. '
             'Defaults to half the image height.'
    )
    camera_info_group.add_argument(
        '--p-fx', type=float, default=None,
        help='X-focal length for populating the projection matrix. '
             'Defaults to the focal length of the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--p-fy', type=float, default=None,
        help='Y-focal length for populating the intrinsic matrix. '
             'Defaults to the focal length of the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--p-cx', type=float, default=None,
        help='Camera center X-coordinate for populating the projection matrix. '
             'Defaults to the value in the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--p-cy', type=float, default=None,
        help='Camera center Y-coordinate for populating the projection matrix. '
             'Defaults to the value in the intrinsic matrix.'
    )
    camera_info_group.add_argument(
        '--p-tx', type=float, default=0.0,
        help='X translation value for populating the projection matrix.'
    )
    camera_info_group.add_argument(
        '--p-ty', type=float, default=0.0,
        help='Y translation value for populating the projection matrix.'
    )
    camera_info_group.add_argument(
        '--roi-x', type=int, default=0, help='Region of interest X-offset.'
    )
    camera_info_group.add_argument(
        '--roi-y', type=int, default=0, help='Region of interest Y-offset.'
    )
    camera_info_group.add_argument(
        '--roi-width', type=int, default=None, help='Region of interest width.'
    )
    camera_info_group.add_argument(
        '--roi-height', type=int, default=None, help='Region of interest height.'
    )
    return parser.parse_args()


def main(args=None):
    args = remove_ros_args(args)
    args = parse_arguments(args)

    rclpy.init()
    node = rclpy.node.Node('publish_image_from_url')
    publisher = node.create_publisher(RosImage, 'image', 10)

    response = requests.get(args.url)
    image = Image.open(BytesIO(response.content))

    encoding = pil_to_sensor_msgs_encoding(image.mode)
    if encoding is None:
        return f"unsupported image encoding '{image.mode}'"

    ros_image = RosImage()
    # Flatten image data into single list
    ros_image.data = list([value for tup in image.getdata() for value in tup])
    ros_image.encoding = encoding
    ros_image.width = image.size[0]
    ros_image.height = image.size[1]
    ros_image.header.frame_id = args.frame_id

    camera_info_publisher = None
    camera_info_message = None
    if args.camera_info:
        camera_info_publisher = node.create_publisher(CameraInfo, 'camera_info', 10)
        camera_info_message = CameraInfo()
        camera_info_message.header.frame_id = args.frame_id
        camera_info_message.width = ros_image.width
        camera_info_message.height = ros_image.height
        camera_info_message.k[0] = args.fx
        camera_info_message.k[2] = args.cx if args.cx is not None else ros_image.width / 2
        camera_info_message.k[4] = args.fy
        camera_info_message.k[5] = args.cy if args.cy is not None else ros_image.height / 2

        camera_info_message.p[0] = args.p_fx if args.p_fx is not None else camera_info_message.k[0]
        camera_info_message.p[2] = args.p_cx if args.p_cx is not None else camera_info_message.k[2]
        camera_info_message.p[3] = args.p_tx
        camera_info_message.p[5] = args.p_fy if args.p_fy is not None else camera_info_message.k[4]
        camera_info_message.p[6] = args.p_cy if args.p_cy is not None else camera_info_message.k[5]
        camera_info_message.p[7] = args.p_ty
        camera_info_message.p[10] = 1.0

        camera_info_message.roi.x_offset = args.roi_x
        camera_info_message.roi.y_offset = args.roi_y
        camera_info_message.roi.width = \
            args.roi_width if args.roi_width is not None else ros_image.width
        camera_info_message.roi.height = \
            args.roi_height if args.roi_height is not None else ros_image.height

    show_thread = None
    if args.show:
        show_thread = threading.Thread(target=image.show)
        show_thread.start()

    clock = node.get_clock()
    last_publish_time = clock.now()
    rclpy_period = rclpy.duration.Duration(seconds=args.period)
    try:
        while rclpy.ok():
            now_time = clock.now()
            dt = now_time - last_publish_time
            if dt >= rclpy_period:
                now_time_msg = now_time.to_msg()
                ros_image.header.stamp = now_time_msg
                publisher.publish(ros_image)
                if camera_info_publisher is not None:
                    camera_info_message.header.stamp = now_time_msg
                    camera_info_publisher.publish(camera_info_message)
                last_publish_time = now_time
                dt = rclpy_period
            clock.sleep_until(now_time + dt)
    except KeyboardInterrupt:
        pass

    if show_thread is not None:
        show_thread.join()


def pil_to_sensor_msgs_encoding(pil_mode) -> Optional[Text]:
    """
    Given an PIL image mode, return the corresponding sensor_msgs image encoding.
    String codes for sensor_msgs/msg/Image can be found at
    https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
    String codes for PIL images can be found at
    https://pillow.readthedocs.io/en/stable/handbook/concepts.html#modes
    :return: The string code for the image encoding or `None` if the mode is not supported.
    """
    if pil_mode == 'L':
        return 'mono8'
    elif pil_mode == 'RGB':
        return 'rgb8'
    elif pil_mode == 'RGBA':
        return 'rgba8'
    elif pil_mode == 'I':
        return '32SC3'
    elif pil_mode == 'F':
        return '32FC3'
    return None


if __name__ == '__main__':
    sys.exit(main())
