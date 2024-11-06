Feature: Check for bsn features

	Scenario: BSN-P11 If data has been sent by the sensor node, the BodyHub is able to process it as low, moderate or high risk vital sign data.
		Given the Target System Data topic is online
		When I listen to topics:
			| Topic Name           |
			|  thermometer data    |
			|  ecg data            |
			|  Target System Data  |
		Then sensors will process the risks
		And Target System Data will receive the risks from sensors and detect patient's status

	Scenario: BSN-P09 If data has been sent by the sensor node, the BodyHub is able to process it
		Given the Target System Data topic is online
		When I listen to topics:
			| Topic Name          |
			| thermometer data    |
			| ecg data            |
			| Target System Data  |
		Then sensors will process the data
		And Target System Data will receive the data from sensors
