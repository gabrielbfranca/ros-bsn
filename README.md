Background
This is a ROS2 System based on Humble Version. It is designed to run on Linux. 
It aims to simulate the transmission of data from several sensors to a receiving hub in a medical setting.

Implementation 
A sensor is a publisher node, which publishes to 2 topics:
1. registration_status_topic: the data transmited consists of 1. integer (called "num", which corresponds to sensor_id, and 2. boolean value (called "data", which corresponds to whether the sensor is active or not)
2. health_data_topic: the data transmitted consists of 1. integer (called "num", which corresponds to sensor_id, and 2. string (called "data", which corresponds to the measured data in a sensor).

A receiving hub is a subscriber node, which subscribes to the 2 topics.
It then saves each topic to an array called "array_registration_status_" "array_health_data_" and respectively. Each array contains data from all the sensors. Data corresponding to a unique sensor can be accessed using their unique sensor_id, with the help of associative map. 

The sensor and the hub is in the package "system". 
The sensor is written in file "sensor.cpp", while the hub is written in file "hub.cpp".

The format of the data transmitted for the 2 topics is written in a separate package "format_data".  The format of the data for the registration_status_topic is in "Registration.msg" file, while the format of the data for the health_data_topic is in "Data.msg" file.

Execution
Step 1: Build the respective packages
In the root of your workspace (eg ~/ros2_ws), run the following command:
colcon build --packages-select format_data

Open a new terminal (also at the root of your workspace (eg ~/ros2_ws)), run the following command:
source install/setup.bash
colcon build --packages-select system

Step 2: Run the hub node 
Open a new terminal (also at the root of your workspace (eg ~/ros2_ws)), run the following command:
source install/setup.bash
ros2 run system hub

Step 3: Run the sensor node 
Open a new terminal (also at the root of your workspace (eg ~/ros2_ws)), run the following command:
source install/setup.bash
ros2 run system sensor
