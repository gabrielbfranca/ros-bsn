Feature: Test Hub Node integration with sensors
  Scenario: Sensors publish to topics and hub_node receives the messages
    Given the ROS 2 system is running with hub_node and sensors
    When sensor_1 and sensor_2 publish to /health_data_topic and /registration_status_topic
    Then hub_node should receive the messages on /health_data_topic and /registration_status_topic
