Feature: ensure /hub_node connects with /sensor_1 and /sensor_2
	Scenario: Ensure that /hub_node is connected to /sensor_1,/sensor_2
		Given the /hub_node node is online
		When I check if node /sensor_1 publishes to /health_data_topic,/registration_status_topic
		Then /hub_node should have /health_data_topic,/registration_status_topic subscribed 
