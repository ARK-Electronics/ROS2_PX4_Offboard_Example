# Topic Mapping
This file keeps track of the topic mapping project between PX4 and ROS2. The implementation can be found in [foxglove.py](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example/blob/FoxgloveTest/px4_offboard/px4_offboard/foxglove.py). We're using Foxglove Studio to visualize the data.

Each topic should have it's definition attached as a hyperlink to the PX4 repo and the ROS2 repo.

Each topic converion below has PX4 on the left and ROS2 on the right.

 PX4 -> ROS2

### DDS Topics File
You will need to take the dds_topics.yaml file and replace it with the file of the same name in your PX4-Autopilot installation. It's location can be found [here](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml). This change gives the DDS bridge the ability to publish new topics such as VehicleAcceleration. Without it, we could not convert this information to ROS2.


## Topics
### Implemented

[vehicle_gps_position(SensorGps.msg)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGps.msg) -> [NavSatFix.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatFix.msg)

[VehicleAcceleration.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAcceleration.msg) -> [Accel.msg](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Accel.msg)(Requires changes to [dds_topics.yaml](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/uxrce_dds_client/dds_topics.yaml))

### TODO




[BatteryStatus.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/BatteryStatus.msg)-> [BatteryState.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/BatteryState.msg)

[VehicleImu.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleImu.msg) -> [Imu.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)

[VehicleMagnetometer.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleMagnetometer.msg) -> [MagneticField.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/MagneticField.msg)



[vehicle_gps_position(SensorGps.msg)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGps.msg) -> [NavSatStatus.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/NavSatStatus.msg) ?

[vehicle_gps_position(SensorGps.msg)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGps.msg) -> [TimeReference.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/TimeReference.msg) ?

[DistanceSensor.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DistanceSensor.msg) -> [Range.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Range.msg)

[SensorHygrometer.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorHygrometer.msg) -> [RelativeHumidity.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/RelativeHumidity.msg) ?

[VehicleAirData.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleAirData.msg) -> [Temperature.msg](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Temperature.msg)

## Issues

[DebugArray.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugArray.msg) -> [DiagnosticArray.msg](https://github.com/ros2/common_interfaces/blob/humble/diagnostic_msgs/msg/DiagnosticArray.msg) (DDS isn't sending this from PX4. When 'ros2 topic list' is run the topic is not listed.)

[DebugValue.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/DebugValue.msg) -> [DiagnosticStatus.msg](https://github.com/ros2/common_interfaces/blob/humble/diagnostic_msgs/msg/DiagnosticStatus.msg)(DDS isn't sending this from PX4. When 'ros2 topic list' is run the topic is not listed.)

