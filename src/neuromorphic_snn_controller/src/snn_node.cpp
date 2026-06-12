#include <chrono>
#include <memory>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class SNNControllerNode : public rclcpp::Node
{
public:
  SNNControllerNode()
  : Node("snn_controller_node"), tau_(0.9), v_th_(1.0), v_reset_(0.0), total_spikes_(0)
  {
    // Publishers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&SNNControllerNode::scan_callback, this, _1));
    phero_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "stigmergy/pheromone_level", 10, std::bind(&SNNControllerNode::phero_callback, this, _1));

    // SNN Network Init (e.g., 9 inputs: 8 lidar + 1 pheromone, 20 hidden, 2 output)
    num_inputs_ = 9;
    num_hidden_ = 20;
    num_outputs_ = 2;
    
    v_hidden_.resize(num_hidden_, 0.0);
    v_out_.resize(num_outputs_, 0.0);
    
    // Fixed heuristic weights for demo (would normally load from trained model)
    w_in_hidden_.resize(num_hidden_, std::vector<float>(num_inputs_, 0.1f));
    w_hidden_out_.resize(num_outputs_, std::vector<float>(num_hidden_, 0.1f));

    // Timer for simulation steps (e.g. 10Hz)
    timer_ = this->create_wall_timer(
      100ms, std::bind(&SNNControllerNode::step_network, this));
      
    RCLCPP_INFO(this->get_logger(), "Neuromorphic SNN Controller Node Started.");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = *msg;
  }

  void phero_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_phero_ = msg->data;
  }

  void step_network()
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. Prepare Inputs
    std::vector<float> input_rates(num_inputs_, 0.0f);
    if (!latest_scan_.ranges.empty()) {
       for (int i=0; i<8; i++) {
           input_rates[i] = 1.0f; // Mock implementation
       }
    }
    input_rates[8] = latest_phero_;
    
    // 2. Compute LIF Dynamics
    float dt = 0.1f;
    std::vector<bool> input_spikes(num_inputs_, false);
    for (int i = 0; i < num_inputs_; ++i) {
        input_spikes[i] = (static_cast<float>(rand()) / RAND_MAX) < (input_rates[i] * dt * 10.0f);
    }
    
    std::vector<bool> hidden_spikes(num_hidden_, false);
    for (int i = 0; i < num_hidden_; ++i) {
        float input_sum = 0.0f;
        for (int j = 0; j < num_inputs_; ++j) {
            if (input_spikes[j]) input_sum += w_in_hidden_[i][j];
        }
        v_hidden_[i] = v_hidden_[i] * exp(-dt / tau_) + input_sum;
        if (v_hidden_[i] >= v_th_) {
            hidden_spikes[i] = true;
            v_hidden_[i] = v_reset_;
        }
    }
    
    std::vector<bool> motor_spikes(num_outputs_, false);
    for (int i = 0; i < num_outputs_; ++i) {
        float hidden_sum = 0.0f;
        for (int j = 0; j < num_hidden_; ++j) {
            if (hidden_spikes[j]) hidden_sum += w_hidden_out_[i][j];
        }
        v_out_[i] = v_out_[i] * exp(-dt / tau_) + hidden_sum;
        if (v_out_[i] >= v_th_) {
            motor_spikes[i] = true;
            v_out_[i] = v_reset_;
            total_spikes_++;
        }
    }
    
    // 3. Decode Spikes to Velocity Command
    auto twist = geometry_msgs::msg::Twist();
    float base_speed = 0.5f;
    twist.linear.x = base_speed;
    if (motor_spikes[0] && !motor_spikes[1]) twist.angular.z = 1.0f; // Left
    if (motor_spikes[1] && !motor_spikes[0]) twist.angular.z = -1.0f; // Right
    
    cmd_vel_pub_->publish(twist);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    
    RCLCPP_INFO(this->get_logger(), "SNN Step Time: %ld us | Total Spikes: %lu", duration.count(), total_spikes_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr phero_sub_;
  
  sensor_msgs::msg::LaserScan latest_scan_;
  float latest_phero_ = 0.0f;

  int num_inputs_, num_hidden_, num_outputs_;
  float tau_, v_th_, v_reset_;
  unsigned long total_spikes_;
  
  std::vector<float> v_hidden_, v_out_;
  std::vector<std::vector<float>> w_in_hidden_, w_hidden_out_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SNNControllerNode>());
  rclcpp::shutdown();
  return 0;
}
