#include <rclcpp/rclcpp.hpp>
#include <format_data/msg/data.hpp>
#include <format_data/msg/registration.hpp>
#include <fstream>
#include <vector>
#include <string>

class Sensor : public rclcpp::Node
{
public:
  Sensor(const std::string &sensor_name, bool status, const std::string &filename, const rclcpp::NodeOptions &options)
      : Node("sensor_node_" + sensor_name, options), line_index_(0), status_(status), sensorName(sensor_name)
  {
    // Create publishers for 2 topics. 1. registration_status_topic; 2. health_data_topic
    registration_status_publisher_ = this->create_publisher<format_data::msg::Registration>("registration_status_topic", 10);
    std::string health_data_topic;
    if (sensor_name == "trm") {
        health_data_topic = "thermometer_data";
    } else if (sensor_name == "ecg") {
        health_data_topic = "ecg_data";
    } else {
        health_data_topic = "unknown_sensor_data"; // Fallback for unrecognized sensor names
    }

    // Create the health data publisher with the determined topic
    health_data_publisher_ = this->create_publisher<format_data::msg::Data>(health_data_topic, 10);

    // Read the file contents
    read_file(filename, lines_);

    // Publish the registration_status_topic (sensor name and status)
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&Sensor::publish_registration_status, this));

    // Publish the health_data_topic only if status is true
    if (status_)
    {
      timer_ = this->create_wall_timer(
          std::chrono::seconds(1),
          std::bind(&Sensor::publish_health_data, this));
    }

    // Configure risk parameters based on sensor name
    highRisk0 = sensor_name + "_HighRisk0";
    highRisk1 = sensor_name + "_HighRisk1";
    midRisk0 = sensor_name + "_MidRisk0";
    midRisk1 = sensor_name + "_MidRisk1";
    lowRisk = sensor_name + "_LowRisk";

    declare_parameter(highRisk0.c_str(), std::vector<double>{0});
    declare_parameter(highRisk1.c_str(), std::vector<double>{0});
    declare_parameter(midRisk0.c_str(), std::vector<double>{0});
    declare_parameter(midRisk1.c_str(), std::vector<double>{0});
    declare_parameter(lowRisk.c_str(), std::vector<double>{0});
  }

private:
  void read_file(const std::string &filename, std::vector<double> &lines)
  {
    std::ifstream file(filename);
    if (!file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line))
    {
      if (!line.empty())
      {
        //RCLCPP_INFO_STREAM(this->get_logger(), "Read from file: " << line);
        lines.push_back(std::stod(line));
      }
    }

    file.close();
  }
  void publish_health_data()
  {
    if (status_)
    {
      if (line_index_ < lines_.size())
      {
        auto message = format_data::msg::Data();
        try
        {
          message.type = sensorName;  // Use sensor name instead of numeric ID
          message.data = lines_[line_index_++];
          message.risk = calculateRisk(message.data);
          RCLCPP_INFO_STREAM(this->get_logger(), "Publishing to health_data_topic: " << message.type << ", " << message.data << ", " << message.risk);
          health_data_publisher_->publish(message);
        }
        catch (const std::invalid_argument &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Invalid data format in file for health data topic %s", e.what());
          return;
        }
      }
      else
      {
        line_index_ = 0;
        RCLCPP_INFO_STREAM(this->get_logger(), "End of file reached. Resetting to the beginning.");
      }
    }
  }

  void publish_registration_status()
  {
    auto message = format_data::msg::Registration();
    message.type = sensorName;  
    message.data = status_ ? "true" : "false";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing registration status: " << message.type << ", " << message.data);
    registration_status_publisher_->publish(message);
  }


  double calculateRisk(double value) {
    auto highRisk0Values = get_parameter(highRisk0.c_str()).as_double_array();
    auto highRisk1Values = get_parameter(highRisk1.c_str()).as_double_array();
    auto midRisk0Values = get_parameter(midRisk0.c_str()).as_double_array();
    auto midRisk1Values = get_parameter(midRisk1.c_str()).as_double_array();
    auto lowRiskValues = get_parameter(lowRisk.c_str()).as_double_array();

    double risk;

    if(value >= highRisk0Values[0] && value <= highRisk0Values[1]) {
      // 66% a 100%
      risk = calculateBorderRisk(value, highRisk0Values, false);
      risk = risk * 0.34 + 0.66;
    } else if(value >= highRisk1Values[0] && value <= highRisk1Values[1]) {
      // 66% a 100%
      risk = calculateBorderRisk(value, highRisk1Values, true);
      risk = risk * 0.34 + 0.66;
    } else if(value >= midRisk0Values[0] && value <= midRisk0Values[1]) {
      // 20% a 65%
      risk = calculateBorderRisk(value, midRisk0Values, false);
      risk = risk * 0.45 + 0.2;
    } else if(value >= midRisk1Values[0] && value <= midRisk1Values[1]) {
      // 20% a 65%
      risk = calculateBorderRisk(value, midRisk1Values, true);
      risk = risk * 0.45 + 0.2;
    } else if(value >= lowRiskValues[0] && value <= lowRiskValues[1]) {
      // 0% a 20%
      risk = calculateCentralRisk(value, lowRiskValues);
      risk = risk * 0.2;
    } else {
      throw std::invalid_argument("Value not in any range");
    }
    return risk;
  }

  double calculateCentralRisk(double value, std::vector<double> values) {
    double min_value = values[0];
    double max_value = values[1];
    double range = max_value - min_value;
    double center = (min_value + max_value) / 2;
    double distance = abs(center - value);
    double risk = distance / range;
    return risk;    
  }

  double calculateBorderRisk(double value, std::vector<double> values, bool increasing) {
    double min_value = values[0];
    double max_value = values[1];
    double range = max_value - min_value;
    
    double risk;
    if(increasing) {
      risk = (value - min_value) / range;
    } else {
      risk = (max_value - value) / range;
    }
    return risk;
  }

  std::vector<double> lines_;
  size_t line_index_;
  bool status_;
  rclcpp::Publisher<format_data::msg::Data>::SharedPtr health_data_publisher_;
  rclcpp::Publisher<format_data::msg::Registration>::SharedPtr registration_status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::string sensorName;
  std::string highRisk0, highRisk1, midRisk0, midRisk1, lowRisk;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // initialize 2 publisher nodes
  // std::string file_path_1 = "/home/windsurff/ros0_ws/src/system/src/number.txt";
  std::string file_path_1 = "/home/ws/src/system/src/temperature_data.txt";

  
  auto options_1 = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=thermometer_sensor"});
  auto node_1 = std::make_shared<Sensor>("trm", true, file_path_1, options_1);

  // std::string file_path_2 = "/home/windsurff/ros0_ws/src/system/src/numberstring.txt";
  std::string file_path_2 = "/home/ws/src/system/src/heart_rate.txt";

  
  auto options_2 = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__node:=ecg_sensor"});
  auto node_2 = std::make_shared<Sensor>("ecg", true, file_path_2, options_2);

  // Create an executor to manage both nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_1);
  executor.add_node(node_2);

  // Spin the executor to allow both nodes to run concurrently
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
