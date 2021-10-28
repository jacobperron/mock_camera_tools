# Copyright 2021 Jacob Perron
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

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


class MockCameraInfoNode(Node):
    """Publishes mock camera info messages."""

    def __init__(self):
        super().__init__('mock_camera_info')
        self._publisher = self.create_publisher(CameraInfo, 'camera_info', 10)
        self._camera_info_msg = CameraInfo()
        self._camera_info_msg.p = [
            1.0, 0.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        def image_callback(image_msg):
            self._camera_info_msg.header = image_msg.header
            self._camera_info_msg.width = image_msg.width
            self._camera_info_msg.height = image_msg.height
            self._publisher.publish(self._camera_info_msg)

        self._subscription = self.create_subscription(Image, 'image', image_callback, 10)


def main(args=None):
    rclpy.init(args=args)

    node = MockCameraInfoNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
