Feature: ensure /hub_node connects with /sensor_node_1 and /sensor_node_2

	Scenario: Ensure that /hub_node is connected to /sensor_node_1,/sensor_node_2
		Given the /hub_node node is online
		When I check if topics 
		Then /injector node is connected appropriately