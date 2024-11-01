#include <functional>
#include <memory>
#include <numeric>
#include <vector>
#include <string>
#include <unordered_map>  // Include for storing sensor data in maps

#include "rclcpp/rclcpp.hpp"
#include <format_data/msg/data.hpp>
#include <format_data/msg/registration.hpp>
#include <format_data/msg/target_system_data.hpp> // TargetSystemData

using std::placeholders::_1;

class MultiTopicSubscriber : public rclcpp::Node
{
public:
  MultiTopicSubscriber()
  : Node("multi_topic_subscriber")
  {
    // Subscription to health_data_topic
        subscribeToHealthData("trm_data");
        subscribeToHealthData("ecg_data");

    // Subscription to registration_status_topic
    subscription_status_ = this->create_subscription<format_data::msg::Registration>(
      "registration_status_topic", 10, std::bind(&MultiTopicSubscriber::registration_status_callback, this, _1));

    publisher_ = this->create_publisher<format_data::msg::TargetSystemData>("TargetSystemData", 10);
  }

private:
  void subscribeToHealthData(const std::string &topic_name) {
        auto subscription = this->create_subscription<format_data::msg::Data>(
            topic_name, 10, std::bind(&MultiTopicSubscriber::health_data_callback, this, _1));
        subscriptions_.emplace_back(std::move(subscription)); // Store the subscription
    }
  std::vector<std::string> getPatientStatus() { 
    std::string sensor_risk_str;
    std::string abps;
    std::string abpd;
    std::string oxi;
    std::string ecg;
    std::string trm;
    std::string glc;

    // Using string keys "trm" and "ecg" to access array_health_data_
    std::vector<std::string> sensor_keys = {"trm", "ecg"};

    for (const auto &key : sensor_keys) {
        if (!array_health_data_[key].empty()) {  // Ensure the vector is not empty
            format_data::msg::Data msg = array_health_data_[key].back();
            double sensor_risk = msg.risk;

            if (sensor_risk > 0 && sensor_risk <= 20) {
                sensor_risk_str = "low risk";
            } else if (sensor_risk > 20 && sensor_risk <= 65) {
                sensor_risk_str = "moderate risk";
            } else if (sensor_risk > 65 && sensor_risk <= 100) {
                sensor_risk_str = "high risk";
            } else {
                sensor_risk_str = "unknown";
            }

            if (key == "trm") {
                trm = sensor_risk_str;
                trm_risk_ = sensor_risk;
            } else if (key == "ecg") {
                ecg = sensor_risk_str;
                ecg_risk_ = sensor_risk;
            }
        }
    }

    std::vector<std::string> v = {trm, ecg, oxi, abps, abpd, glc};
    return v;
}


  // BSN's process
  void process(){
    std::vector<double> current_data;

    for(auto it = array_health_data_.begin(); it != array_health_data_.end(); it++) {
      double el = it->second[0].risk;
      current_data.push_back(el);
      if(it->second.size() > 1) it->second.erase(it->second.begin());
    }

    patient_status_ = data_fuse(current_data); // consumes 1 packt per sensor (in the buffers that have packages to array_health_data_ be processed)
    // RCLCPP_INFO(this->get_logger(), "patient_status_ = %.2lf", patient_status_);

    getPatientStatus();

    std::string patient_risk;

    if(patient_status_ <= 20) {
      patient_risk = "VERY LOW RISK";
    } else if(patient_status_ > 20 && patient_status_ <= 40) {
      patient_risk = "LOW RISK";
    } else if(patient_status_ > 40 && patient_status_ <= 60) {
      patient_risk = "MODERATE RISK";
    } else if(patient_status_ > 60 && patient_status_ <= 80) {
      patient_risk = "CRITICAL RISK";
    } else if(patient_status_ > 80 && patient_status_ <= 100) {
      patient_risk = "VERY CRITICAL RISK";
    }

    RCLCPP_INFO(this->get_logger(), "*****************************************");
    RCLCPP_INFO(this->get_logger(), "PatientStatusInfo#");
    RCLCPP_INFO(this->get_logger(), "| THERM_RISK: %s", std::to_string(trm_risk_).c_str());
    RCLCPP_INFO(this->get_logger(), "| ECG_RISK: %s", std::to_string(ecg_risk_).c_str());
    // RCLCPP_INFO(this->get_logger(), "| OXIM_RISK: %s", std::to_string(oxi_risk).c_str());
    // RCLCPP_INFO(this->get_logger(), "| ABPS_RISK: %s", std::to_string(abps_risk).c_str());
    // RCLCPP_INFO(this->get_logger(), "| ABPD_RISK: %s", std::to_string(abpd_risk).c_str());
    // RCLCPP_INFO(this->get_logger(), "| GLC_RISK: %s", std::to_string(glc_risk).c_str());
    RCLCPP_INFO(this->get_logger(), "| PATIENT_STATE: %s", patient_risk.c_str());
    RCLCPP_INFO(this->get_logger(), "*****************************************");
    transfer();
  }

  // BSN's transfer
  void transfer() {
    format_data::msg::TargetSystemData msg;

    // msg.trm_batt = trm_batt;
    // msg.ecg_batt = ecg_batt;

    msg.trm_risk = trm_risk_;
    msg.ecg_risk = ecg_risk_;
    msg.trm_data = trm_raw_;
    msg.ecg_data = ecg_raw_;

    msg.patient_status = patient_status_;

    publisher_->publish(msg);
  }

  // Data fusion function from BSN::Processor
  double data_fuse(std::vector<double> packetsReceived) {	
    double average, risk_status;
    int32_t count = 0;
    average = 0;
    int32_t index = 0;
    double bpr_avg = 0.0;
    
    std::vector<double> values;
    std::vector<double>::iterator packets_it;
    for(packets_it = packetsReceived.begin();packets_it != packetsReceived.end();++packets_it){
      if(static_cast<int>(*packets_it) >= 0)  {
        // Soma à média e retira da fila
        if(index == 3 || index == 4) {
          bpr_avg += *packets_it;
        } else {
          average += *packets_it;
          values.push_back(*packets_it);
        }

        count++;
      }

      if(index == 4) {
        if(bpr_avg >= 0.0) {
          bpr_avg /= 2;
          average += bpr_avg;
          values.push_back(bpr_avg);
        }
      }

      index++;
    }

    if(count == 0)
      return -1;

    // Calcula a media partir da soma dividida pelo número de pacotes lidos
    double avg = (average / count);

    std::vector<double> deviations;
    double min = 1000; //Maior valor possível é 100
    double max = -1; //Menor valor possível é 0

    size_t i;

    for(i = 0;i < values.size();i++) {
      //Cálculo dos desvios individuais
      double dev;
      dev = values.at(i) - avg;

      deviations.push_back(dev);

      if(dev > max) {
        max = dev;
      }

      if (dev < min) {
        min = dev;
      }
    }

    double weighted_average = 0.0;
    double weight_sum = 0.0;

    // Status de risco do paciente dado em porcentagem
    if(max - min > 0.0) {
      //Se o máximo e mínimo forem diferentes, normalizar desvios e calcular média ponderada
      for(i = 0;i < deviations.size();i++) {
        //Normalizando desvios entre 0 e 1
        deviations.at(i) = (deviations.at(i) - min)/(max - min);

        weight_sum += deviations.at(i);
        weighted_average += values.at(i)*deviations.at(i);
      }

      risk_status = weighted_average/weight_sum;
    } else {
      //Se o máximo é igual ao mínimo, a média será calculada e dará o mesmo valor
      risk_status = avg;
    }

    return risk_status;
  }

  // Callback for health_data_topic
  void health_data_callback(const format_data::msg::Data& msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Received message from health_data_topic: type = '%s', data = '%.2lf', risk = %.2lf", msg.type.c_str(), msg.data, msg.risk*100);
    
    // Save received message to the corresponding array based on sensor ID (msg.num)
    array_health_data_[msg.type].push_back(msg);
    
    //print_array("health_data_topic", array_health_data_[msg.num], msg.num);

    if (msg.type == "trm") {
      trm_raw_ = msg.data;
    } else if (msg.type == "ecg") {
      ecg_raw_ = msg.data;
    }
    if (array_health_data_["trm"].size() > 2 && array_health_data_["ecg"].size() > 2) {
      process();
     }
  }

  // Callback for registration_status
  void registration_status_callback(const format_data::msg::Registration & msg) 
  {
    RCLCPP_INFO(this->get_logger(), "Received message from registration_status: sensor_type = '%s', status = '%s'", msg.type.c_str(), msg.data ? "true" : "false");
    
    // Save received message to the corresponding array based on sensor ID (msg.num)
    array_registration_status_[msg.type].push_back(msg);
    
    //print_array("registration_status", array_registration_status_[msg.num], msg.num);
  }

  // Function to print all entries of an array of health_data_topic for a specific sensor ID
  /*
  void print_array(const std::string &topic_name, const std::vector<format_data::msg::Data> &data_array, int32_t sensor_id) const
  {
    RCLCPP_INFO(this->get_logger(), "Contents of %s for sensor ID %d:", topic_name.c_str(), sensor_id);
    for (const auto &msg : data_array)
    {
      RCLCPP_INFO(this->get_logger(), "  num = '%d', data = '%.2lf'", msg.num, msg.data);
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
  */
  // Maps to store received messages by sensor ID
  std::vector<std::shared_ptr<rclcpp::Subscription<format_data::msg::Data>>> subscriptions_; // Declare subscriptions
  std::unordered_map<std::string, std::vector<format_data::msg::Data>> array_health_data_;
  std::unordered_map<std::string, std::vector<format_data::msg::Registration>> array_registration_status_;

  // Publisher for CentralHub
  rclcpp::Publisher<format_data::msg::TargetSystemData>::SharedPtr publisher_;

  // Subscriptions for each topic
  rclcpp::Subscription<format_data::msg::Data>::SharedPtr subscription_health_data_;
  rclcpp::Subscription<format_data::msg::Registration>::SharedPtr subscription_status_;

  // Sensor data
  double trm_risk_;
  double ecg_risk_;
  double trm_raw_;
  double ecg_raw_;
  double patient_status_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiTopicSubscriber>());
  rclcpp::shutdown();
  return 0;
}

