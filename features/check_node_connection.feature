Feature: ensure /hub_node connects with /sensor_node_1 and /sensor_node_2

	Scenario: Ensure that /hub_node is connected to /sensor_node_1,/sensor_node_2
		Given the /hub_node node is online
  		And the /sensor_node_1 node is online
    		And the /sensor_node_2 node is online
		When I check if node /sensor_node_1 publishes to /health_data_topic,/registration_status_topic
  		And I check if node /sensor_node_2 publishes to /health_data_topic,/registration_status_topic
		Then /hub_node should have /health_data_topic,/registration_status_topic subscribed 
