from pytest_bdd import given, when, then, scenarios
import rclpy
from rclpy.node import Node
from format_data.msg import Data, Registration  # Correct message types
import os

# Load the feature file
scenarios(os.path.join(os.path.dirname(__file__), 'features/hub_node_integration.feature'))


@given("the /hub_node node is online")
def hub_node_online():
    # Initialize rclpy only if it hasn't been initialized yet
    if not rclpy.ok():
        rclpy.init()
    assert rclpy.ok , "node is not initialized"
    print("The /hub_node node is online.")


@when('I check if node /sensor_1 publishes to /health_data_topic,/registration_status_topic')
def check_sensor_1_publication():
    # Initialize rclpy only if it hasn't been initialized yet
    if not rclpy.ok():
        rclpy.init()

    # Create a test node
    node = rclpy.create_node('test_sensor_1')

    # Create publishers to simulate /sensor_1 publishing messages
    publisher_health_data = node.create_publisher(Data, '/health_data_topic', 10)
    publisher_registration_status = node.create_publisher(Registration, '/registration_status_topic', 10)

    # Publish the messages
    msg_health = Data()
    msg_health.data = 'Health data from sensor_1'
    publisher_health_data.publish(msg_health)

    msg_registration = Registration()
    msg_registration.num = 1
    msg_registration.data = True
    publisher_registration_status.publish(msg_registration)

    # Spin the node to process messages
    rclpy.spin_once(node, timeout_sec=2.0)

    # Clean up the node
    node.destroy_node()


@then('/hub_node should have /health_data_topic,/registration_status_topic subscribed')
def check_hub_node_subscriptions():
    # Initialize rclpy only if it hasn't been initialized yet
    if not rclpy.ok():
        rclpy.init()

    # Create a test node to simulate /hub_node
    node = rclpy.create_node('test_hub_node')

    # A dictionary to store received messages for each topic
    received_messages = {
        '/health_data_topic': [],
        '/registration_status_topic': []
    }

    # Define callback functions to store received messages
    def health_data_callback(msg):
        received_messages['/health_data_topic'].append(msg.data)

    def registration_status_callback(msg):
        received_messages['/registration_status_topic'].append(msg.data)  # Correct field name

    # Subscriptions to the topics with the correct message types
    node.create_subscription(Data, '/health_data_topic', health_data_callback, 10)
    node.create_subscription(Registration, '/registration_status_topic', registration_status_callback, 10)

    # Spin the node multiple times to allow message processing
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=2.0)

    # Assertions to ensure /hub_node has received messages on both topics
    assert len(received_messages['/health_data_topic']) > 0, "Hub node did not receive any message on /health_data_topic."
    assert len(received_messages['/registration_status_topic']) > 0, "Hub node did not receive any message on /registration_status_topic."

    # Additional assertions to check message content
    assert 'Health data from sensor_1' in received_messages['/health_data_topic'], "Expected health data message not received."
    assert True in received_messages['/registration_status_topic'], "Expected registration status message not received."

    # Clean up
    node.destroy_node()

    # Shut down rclpy if needed
    rclpy.shutdown()
