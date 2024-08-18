#include <functional>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>  // Include for storing sensor data in maps

#include "rclcpp/rclcpp.hpp"
#include <format_data/msg/data.hpp>
#include <format_data/msg/registration.hpp>

using std::placeholders::_1;

class MultiTopicSubscriber : public rclcpp::Node
{
public:
  MultiTopicSubscriber()
  : Node("multi_topic_subscriber")
  {
    // Subscription to health_data_topic
    subscription_health_data_ = this->create_subscription<format_data::msg::Data>(
      "health_data_topic", 10, std::bind(&MultiTopicSubscriber::health_data_callback, this, _1));

    // Subscription to registration_status_topic
    subscription_status_ = this->create_subscription<format_data::msg::Registration>(
      "registration_status_topic", 10, std::bind(&MultiTopicSubscriber::registration_status_callback, this, _1));
  }

private:
  // Callback for health_data_topic
  void health_data_callback(const format_data::msg::Data& msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Received message from health_data_topic: num = '%d', data = '%s'", msg.num, msg.data.c_str());
    
    // Save received message to the corresponding array based on sensor ID (msg.num)
    array_health_data_[msg.num].push_back(msg);
    
    print_array("health_data_topic", array_health_data_[msg.num], msg.num);
  }

  // Callback for registration_status
  void registration_status_callback(const format_data::msg::Registration & msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Received message from registration_status: sensor_id = '%d', status = '%s'", msg.num, msg.data ? "true" : "false");
    
    // Save received message to the corresponding array based on sensor ID (msg.num)
    array_registration_status_[msg.num].push_back(msg);
    
    print_array("registration_status", array_registration_status_[msg.num], msg.num);
  }

  // Function to print all entries of an array of health_data_topic for a specific sensor ID
  void print_array(const std::string &topic_name, const std::vector<format_data::msg::Data> &data_array, int32_t sensor_id) const
  {
    RCLCPP_INFO(this->get_logger(), "Contents of %s for sensor ID %d:", topic_name.c_str(), sensor_id);
    for (const auto &msg : data_array)
    {
      RCLCPP_INFO(this->get_logger(), "  num = '%d', data = '%s'", msg.num, msg.data.c_str());
    }
  }

  // Overloaded function to print all entries of an array of registration_status_topic for a specific sensor ID
  void print_array(const std::string &topic_name, const std::vector<format_data::msg::Registration> &data_array, int32_t sensor_id) const
  {
    RCLCPP_INFO(this->get_logger(), "Contents of %s for sensor ID %d:", topic_name.c_str(), sensor_id);
    for (const auto &msg : data_array)
    {
      RCLCPP_INFO(this->get_logger(), "  num = '%d', status = '%s'", msg.num, msg.data ? "true" : "false");
    }
  }

  // Maps to store received messages by sensor ID
  std::unordered_map<int32_t, std::vector<format_data::msg::Data>> array_health_data_;
  std::unordered_map<int32_t, std::vector<format_data::msg::Registration>> array_registration_status_;

  // Subscriptions for each topic
  rclcpp::Subscription<format_data::msg::Data>::SharedPtr subscription_health_data_;
  rclcpp::Subscription<format_data::msg::Registration>::SharedPtr subscription_status_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}

