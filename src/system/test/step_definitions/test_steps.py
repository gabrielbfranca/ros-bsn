import pytest
from pytest_bdd import given, when, then, scenarios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Load the scenarios from the feature file
scenarios('../features/hub_node_integration.feature')

# Track node statuses
node_status = {
    'hub_node': False,
    'sensor_1': False,
    'sensor_2': False
}

# Store the messages received by the hub node
received_messages = []


@given("the /hub_node node is online")
def hub_node_online():
    node_status['hub_node'] = True


@given('the /sensor_1 node is online')
def sensor_1_online():
    node_status['sensor_1'] = True


@given('the /sensor_2 node is online')
def sensor_2_online():
    node_status['sensor_2'] = True


@when('I check if node /sensor_1 publishes to /health_data_topic,/registration_status_topic')
def check_sensor_1_publication():
    assert node_status['sensor_1'], "/sensor_1 is not online!"
    
    node = rclpy.create_node('test_sensor_1')
    publisher_health_data = node.create_publisher(String, '/health_data_topic', 10)
    publisher_registration_status = node.create_publisher(String, '/registration_status_topic', 10)

    msg_health = String()
    msg_health.data = 'Health data from sensor_1'
    publisher_health_data.publish(msg_health)

    msg_registration = String()
    msg_registration.data = 'Registration status from sensor_1'
    publisher_registration_status.publish(msg_registration)

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()


@when('I check if node /sensor_2 publishes to /health_data_topic,/registration_status_topic')
def check_sensor_2_publication():
    assert node_status['sensor_2'], "/sensor_2 is not online!"
    
    node = rclpy.create_node('test_sensor_2')
    publisher_health_data = node.create_publisher(String, '/health_data_topic', 10)
    publisher_registration_status = node.create_publisher(String, '/registration_status_topic', 10)

    msg_health = String()
    msg_health.data = 'Health data from sensor_2'
    publisher_health_data.publish(msg_health)

    msg_registration = String()
    msg_registration.data = 'Registration status from sensor_2'
    publisher_registration_status.publish(msg_registration)

    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()


@then('/hub_node should have /health_data_topic,/registration_status_topic subscribed')
def check_hub_node_subscriptions():
    assert node_status['hub_node'], "/hub_node is not online!"
    
    node = rclpy.create_node('test_hub_node')
    
    def health_data_callback(msg):
        received_messages.append(msg.data)

    def registration_status_callback(msg):
        received_messages.append(msg.data)

    node.create_subscription(String, '/health_data_topic', health_data_callback, 10)
    node.create_subscription(String, '/registration_status_topic', registration_status_callback, 10)

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=1.0)

    assert len(received_messages) >= 4, "Hub node did not receive all messages."
    node.destroy_node()

    rclpy.shutdown()
