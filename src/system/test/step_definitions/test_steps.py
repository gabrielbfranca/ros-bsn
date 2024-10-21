import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pytest_bdd import given, when, then

@pytest.fixture
def ros_setup():
    rclpy.init()
    node = Node("test_node")
    yield node
    rclpy.shutdown()

@given('the ROS 2 system is running with hub_node and sensors', scope='function')
def start_ros_system(ros_setup):
    # You can use a launch file here to launch the nodes
    pass

@when('sensor_1 and sensor_2 publish to /health_data_topic and /registration_status_topic', scope='function')
def sensors_publish(ros_setup):
    # Simulate sensor_1 publishing to /health_data_topic
    health_pub = ros_setup.create_publisher(String, '/health_data_topic', 10)
    reg_pub = ros_setup.create_publisher(String, '/registration_status_topic', 10)

    # Create sample messages
    health_msg = String()
    health_msg.data = "Sensor 1 Health Data"
    
    reg_msg = String()
    reg_msg.data = "Sensor 2 Registration Status"
    
    # Publish messages
    health_pub.publish(health_msg)
    reg_pub.publish(reg_msg)

@then('hub_node should receive the messages on /health_data_topic and /registration_status_topic', scope='function')
def hub_node_receives_messages(ros_setup):
    # Subscribe to /health_data_topic and /registration_status_topic to simulate hub_node's behavior
    received_health_data = None
    received_registration_status = None

    def health_callback(msg):
        nonlocal received_health_data
        received_health_data = msg.data

    def reg_callback(msg):
        nonlocal received_registration_status
        received_registration_status = msg.data

    ros_setup.create_subscription(String, '/health_data_topic', health_callback, 10)
    ros_setup.create_subscription(String, '/registration_status_topic', reg_callback, 10)
    
    # Spin ROS once to ensure messages are received
    for _ in range(5):
        rclpy.spin_once(ros_setup)
    
    assert received_health_data == "Sensor 1 Health Data", "hub_node did not receive health data"
    assert received_registration_status == "Sensor 2 Registration Status", "hub_node did not receive registration status"
