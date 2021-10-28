# mock\_camera\_tools

Scripts useful for mocking cameras.

## Contents

**mock\_camera\_info.py**

Subscribes to an [Image](https://github.com/ros2/common_interfaces/blob/a3a0dde2ba184b01cdc59a3003728906de3240a9/sensor_msgs/msg/Image.msg) topic and publishes mock [CameraInfo](https://github.com/ros2/common_interfaces/blob/a3a0dde2ba184b01cdc59a3003728906de3240a9/sensor_msgs/msg/CameraInfo.msg).
Useful for supplementing image topics that do not provide camera info.

