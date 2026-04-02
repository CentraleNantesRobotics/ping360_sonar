# Using the Ping360 as an Altimeter

Run the `ping360_sonar` node. It provides the raw readings on the topic `/scan_echo`
```
ros2 launch ping360_sonar distance_measure.py
```

Launch the altimeter node. By default, sonar images are not published, but this can be controlled by the parameter `pub_raw_img` and `pub_altimeter_img`
```
ros2 launch altimeter_ping360 altimeter.launch.py
```
Make sure the `angle_sector` and `angle_step` parameters are the same for both nodes.
The parameters were tuned from a rosbag recorded at lake Zurich on 31.03.26, they might need some tuning to work well in other conditions. All parameters have descriptions, refer to them for details.