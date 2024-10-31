Feature: Check for bsn features

	Scenario: BSN-P05 If a sensor reports a health status risk, an emergency will be detected in body hub
		Given the /registration_status_topic topic is online
		When I listen to topics:
			| Topic Name                  | Data Type |
			| /registration_status_topic  | num,data  |

		Then registration will process data
