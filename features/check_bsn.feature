Feature: Check for bsn features

	Scenario: BSN-P11 If data has been sent by the sensor node, the BodyHub is able to process it as low, moderate or high risk vital sign data.
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         | Data Type             |
			| /health_data_topic | num,data,risk  		 |
			| /TargetSystemData  | trm_risk,ecg_risk,oxi_risk,abps_risk,abpd_risk,glc_risk,trm_data,ecg_data,ecg_data,oxi_data,abps_data,abpd_data,glc_data,patient_status   |
		Then sensors will process the risks
		And /TargetSystemData will receive the risks from sensors

	Scenario: BSN-P09 If data has been sent by the sensor node, the BodyHub is able to process it
		Given the /TargetSystemData topic is online
		When I listen to topics:
			| Topic Name         | Data Type       		|
			| /thermometer_data  | num,data,risk   		|
			| /ecg_data          | num,data,risk   		|
			| /TargetSystemData  | messages/SensorData  |
		Then sensors will process the data
		And /TargetSystemData will receive the data from sensors