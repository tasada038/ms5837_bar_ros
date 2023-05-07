# ms5837_bar_ros

[![GitHub stars](https://img.shields.io/github/stars/tasada038/ms5837_bar_ros.svg?style=social&label=Star&maxAge=2592000)](https://github.com/tasada038/ms5837_bar_ros/stargazers/)
[![GitHub forks](https://img.shields.io/github/forks/tasada038/ms5837_bar_ros.svg?style=social&label=Fork&maxAge=2592000)](https://github.com/tasada038/ms5837_bar_ros/network/)
[![GitHub issues](https://img.shields.io/github/issues/tasada038/ms5837_bar_ros.svg)](https://github.com/tasada038/ms5837_bar_ros/issues/)
[![GitHub license](https://img.shields.io/github/license/tasada038/ms5837_bar_ros.svg)](https://github.com/tasada038/ms5837_bar_ros/blob/master/LICENSE)

## Overview

ROS 2 package for Blue Robotics Bar30 and Bar02 High Resolution Depth/Pressure Sensor

**Keywords:** ROS 2, Bar30, Bar02

### License

The source code is released under a [MIT license](LICENSE).

## Requirements
[Bar30 High-Resolution 300m Depth/Pressure Sensor](https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar30-sensor-r1/)

[Bar02 Ultra High Resolution 10m Depth/Pressure Sensor](https://bluerobotics.com/store/sensors-sonars-cameras/sensors/bar02-sensor-r1-rp/)

## Installation

Clone with `--recursive` in order to get the necessary `ms5837-python` library:

```
cd dev_ws/src
git clone -b master --recursive https://github.com/tasada038/ms5837_bar_ros.git
cd ~/dev_ws/src/ms5837_bar_ros/ms5837_bar_ros/ms5837-python/
sudo rm -r *.py
cd ~/dev_ws
colcon build --packages-select ms5837_bar_ros
```

## Run
Publish bar data
```
. install/setup.bash
ros2 run ms5837_bar_ros bar30_node
ros2 run ms5837_bar_ros bar02_node
```

Publish bar data using Rviz2
```
. install/setup.bash
ros2 launch ms5837_bar_ros bar30.launch.py
ros2 launch ms5837_bar_ros bar02.launch.py
```

![bar_rviz_img](img/bar_rviz.png)

## Ping sonar Topics
The topics of the ms5837_bar_ros are as follows.

```
$ ros2 topic list
/bar02/param/gain
/bar02/param/interval
/bar02/param/mode
/bar02/param/speed
/bar02/range
```

- std_msgs.msg Float32: /bar02/param/gain
- std_msgs.msg Float32: /bar02/param/interval
- std_msgs.msg Float32: /bar02/param/mode
- std_msgs.msg Float32: /bar02/param/speed
- sensor_msgs.msg Range: /bar02/range

## Ping sonar Parameters
The parameters for the ms5837_bar_ros are as follows.

```
$ ros2 param list
/bar02_node:
  gain_num
  interval_num
  mode_auto
  scan_lenght
  scan_start
  speed
```

- gain_num parameter range is [0 - 6] (int).
- interval_num range is [50 - 200] (int, ms).
- mode_auto range is [0 or 1].
- scan_lenght range is [2000 - 10000] (int ms). Blue Robotics Inc. default is 2000 mm.
- scan_start range is [30 - 200]. Blue Robotics Inc. default is 100 mm.
- scan_start range is [1050000 - 1550000] (int mm/s).

## License
This action is licensed under the MIT License. This project is originally created by [Blue Robotics](https://github.com/bluerobotics), and maintained continuously by Takumi Asada.

Projects in .gitmodules files are covered by Blue Robotics Inc's MIT License.
Other software components are licensed under this project's license.