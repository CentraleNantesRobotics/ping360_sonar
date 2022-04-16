# ping360_sonar
[![GitHub stars](https://img.shields.io/github/stars/CentraleNantesRobotics/ping360_sonar.svg?style=social&label=Star&maxAge=2592000)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/stargazers/)
[![GitHub forks](https://img.shields.io/github/forks/CentraleNantesRobotics/ping360_sonar.svg?style=social&label=Fork&maxAge=2592000)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/network/)
[![Build Status](https://github.com/CentraleNantesRobotics/ping360_sonar/workflows/ROS%20CI/badge.svg)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/)
[![GitHub issues](https://img.shields.io/github/issues/CentraleNantesRobotics/ping360_sonar.svg)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/issues/)
[![GitHub license](https://img.shields.io/github/license/CentraleNantesRobotics/ping360_sonar.svg)](https://github.com/CentraleNantesRobotics/ping360_sonar/blob/master/LICENSE)
[![Github all releases](https://img.shields.io/github/downloads/CentraleNantesRobotics/ping360_sonar/total.svg)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/releases/)
[![GitHub release](https://img.shields.io/github/release/CentraleNantesRobotics/ping360_sonar.svg)](https://GitHub.com/CentraleNantesRobotics/ping360_sonar/releases/)
[![All Contributors](https://img.shields.io/badge/all_contributors-1-orange.svg?style=flat-square)](#contributors-)
## Overview

A ROS 2 package for the [BlueRobotics] [Ping360] Sonar. This code is being developped for Foxy.

**Keywords:** ROS 2, ping360

### License

The source code is released under a [MIT license](LICENSE).

## Installation

<!--### Download the latest release

Get the latest stable release [here](https://github.com/CentraleNantesRobotics/ping360_sonar/releases/latest).-->

### Building from Source

#### Dependencies

- [Robot Operating System (ROS) 2](http://wiki.ros.org) (middleware for robotics),
- [deployment branch of ping-cpp](https://github.com/GSO-soslab/ping-cpp.git) as a submodule
- [image_transport](http://wiki.ros.org/image_transport) for the C++ node

#### Building

Clone with `--recursive` in order to get the necessary `ping-cpp` library:

	cd ros2_workspace/src
	git clone https://github.com/CentraleNantesRobotics/ping360_sonar.git -b ros2 --recursive
	cd ../..
	colcon build --symlink-install --packages-select ping360_sonar

	
## Nodes

The package provides both C++ and Python nodes, they share the same ROS API and expose the same capabilities.

 - C++ version: `ros2 run ping360_sonar ping360_node`
 - Python: `ros2 run ping360_sonar ping360.py`
 
 While continuously rotating the sonar in a set field of view (defined by the min and max angle parameters in gradians), it may publishes three types of messages:
- The raw response data (the echo intensities for a given angle & range)
- A LaserScan msg with ranges detected using a certain intensity threshold:
- A black and white image using the data received from the sonar. Same as the one generated by the ping viewer.

The rotation of the sonar is only limited by the maximum range (hence the echo maximum duration). Images are updated at this rate even if they are actually published at a lower rate.

## Published Topics 

* **`scan_image`** ([sensor_msgs/Image])

	The generated sonar image in gray level. Each pixel is filled depending on the range and angular step of the sonar.
	This topic can be toggled using the **publish_image** parameter. The C++ node uses [image_transport](http://wiki.ros.org/image_transport) while the Python one publishes raw images.

* **`echo`** ([msg/SonarEcho])

	Publishes the raw sonar data in a custom message:
	
		Header header            #header info
		float32 angle               # the measurement angle [rad]
		uint8 gain  # Sonar Gain
		uint16 number_of_samples 
		uint16 transmit_frequency # [kHz]
		uint16 speed_of_sound # [m/s]
		uint8 range      #  range value [m]
		uint8[] intensities    # intensity data [0-255].  This is the actual data received from the sonar
	
	This topic can be toggled using the **publish_echo** parameter.

* **`scan`** ([sensor_msgs/LaserScan])

	Publishes a LaserScan msg with ranges detected above a certain intensity threshold (0-255). The intensities of the message are scaled down to (0,1).
	This topic can be toggled using the **publish_scan** parameter.

 
## Parameters

The list below corresponds to the output of `ros2 param describe` for all `ping360` parameters:

- `angle_sector`
    - Type: integer
    - Description: Scanned angular sector around sonar heading [degrees]. Will oscillate if not 360
    - Default value: 360 [60..360]
- `angle_step`
    - Type: integer
    - Description: Sonar angular resolution [degrees]
    - Default value: 1 [1..20]
- `baudrate`
    - Type: integer
    - Default value: 115200
- `device`
    - Type: string
    - Default value: /dev/ttyUSB0
- `fallback_emulated`
    - Type: boolean
    - Description: Emulates a sonar if Ping360 cannot be initialized
    - Default value: True
- `frame`
    - Type: string
    - Description: Frame ID of the message headers
    - Default value: sonar
- `frequency`
    - Type: integer
    - Description: Sonar operating frequency [kHz]
    - Default value: 740 [650..850]
- `gain`
    - Type: integer
    - Description: Sonar gain (0 = low, 1 = normal, 2 = high)
    - Default value: 0 [0..2]
- `image_rate`
    - Type: integer
    - Description: Image publishing rate [ms]
    - Default value: 100 [50..2000]
- `image_size`
    - Type: integer
    - Description: Output image size [pixels]
    - Default value: 300 [100..2..1000]
- `publish_echo`
    - Type: boolean
    - Description: Publish raw echo on 'scan_echo'
    - Default value: False
- `publish_image`
    - Type: boolean
    - Description: Publish images on 'scan_image'
    - Default value: True
- `publish_scan`
    - Type: boolean
    - Description: Publish laserscans on 'scan'
    - Default value: False
- `range_max`
    - Type: integer
    - Description: Sonar max range [m]
    - Default value: 2 [1..50]
- `scan_image.format`
    - Type: string
    - Description: Compression method
    - Default value: jpeg
- `scan_image.jpeg_quality`
    - Type: integer
    - Description: Image quality for JPEG format
    - Default value: 95 [1..100]
- `scan_image.png_level`
    - Type: integer
    - Description: Compression level for PNG format
    - Default value: 3 [0..9]
- `scan_threshold`
    - Type: integer
    - Description: Intensity threshold for LaserScan message
    - Default value: 200 [1..255]
- `speed_of_sound`
    - Type: integer
    - Description: Speed of sound [m/s]
    - Default value: 1500 [1450..1550]

Most the parameters can be updated during execution, except `baudrate`, `device`, `frame`, `image_rate` and `fallback_emulated`.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/CentraleNantesRobotics/ping360_sonar/issues).


[ROS]: http://www.ros.org
[BlueRobotics]: http://bluerobotics.com
[Ping360]: https://bluerobotics.com/store/sensors-sonars-cameras/sonar/ping360-sonar-r1-rp/
[Image transport]: http://wiki.ros.org/image_transport
[msg/SonarEcho]: /msg/SonarEcho.msg
[sensor_msgs/LaserScan]: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/LaserScan.msg

## Contributors

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
	  <td align="center"><a href="https://github.com/oKermorgant"><img src="https://avatars.githubusercontent.com/u/1633173?v=4" width="100px;" alt=""/><br /><sub><b>Olivier Kermorgant</b></sub></a><br /><a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=oKermorgant" title="Code">💻</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=oKermorgant" title="Tests">⚠️</a></td>
    <td align="center"><a href="https://github.com/Hameck"><img src="https://avatars2.githubusercontent.com/u/14954732?v=4" width="100px;" alt=""/><br /><sub><b>Henrique Martinez Rocamora</b></sub></a><br /><a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Hameck" title="Code">💻</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Hameck" title="Tests">⚠️</a></td>
    <td align="center"><a href="https://stormix.co"><img src="https://avatars2.githubusercontent.com/u/18377687?v=4" width="100px;" alt=""/><br /><sub><b>Anas Mazouni</b></sub></a><br /><a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Stormiix" title="Code">💻</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Stormiix" title="Tests">⚠️</a></td>
    <td align="center"><a href="https://github.com/tomlogan501"><img src="https://avatars3.githubusercontent.com/u/56969577?v=4" width="100px;" alt=""/><br /><sub><b>tomlogan501</b></sub></a><br /><a href="#ideas-tomlogan501" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=tomlogan501" title="Tests">⚠️</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/issues?q=author%3Atomlogan501" title="Bug reports">🐛</a></td>
	  <td align="center"><a href="https://github.com/AlexisFetet/"><img src="https://avatars.githubusercontent.com/u/94527511?v=4" width="100px;" alt=""/><br /><sub><b>Alexis Fetet</b></sub></a><br /><a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=AlexisFetet" title="Code">💻</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=AlexisFetet" title="Tests">⚠️</a></td>
	  <td align="center"><a href="https://github.com/Foukoo"><img src="https://avatars.githubusercontent.com/u/59455485?v=4" width="100px;" alt=""/><br /><sub><b>Jonathan Delacoux</b></sub></a><br /><a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Foukoo" title="Code">💻</a> <a href="https://github.com/CentraleNantesRobotics/ping360_sonar/commits?author=Foukoo" title="Tests">⚠️</a></td>
  </tr>
</table>

<!-- markdownlint-enable -->
<!-- prettier-ignore-end -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. 

Contributions of any kind welcome! Please refer to our [Contribution Guide](CONTRIBUTING.md)

