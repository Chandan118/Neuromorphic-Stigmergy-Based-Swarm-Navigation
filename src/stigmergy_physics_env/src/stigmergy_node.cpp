#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class StigmergyNode : public rclcpp::Node
{
public:
  StigmergyNode()
  : Node("stigmergy_physics_node"), 
    width_(10.0), height_(10.0), res_(20.0), 
    diffusion_rate_(0.01), evaporation_rate_(0.005)
  {
    nx_ = static_cast<int>(width_ * res_);
    ny_ = static_cast<int>(height_ * res_);
    grid_.resize(ny_, std::vector<float>(nx_, 0.0f));
    next_grid_.resize(ny_, std::vector<float>(nx_, 0.0f));

    phero_pub_ = this->create_publisher<std_msgs::msg::Float32>("stigmergy/pheromone_level", 10);
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&StigmergyNode::odom_callback, this, _1));

    timer_ = this->create_wall_timer(
      100ms, std::bind(&StigmergyNode::step_pde, this));
      
    RCLCPP_INFO(this->get_logger(), "Stigmergy Physics PDE Node Started.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    
    int idx_x = std::max(0, std::min(nx_ - 1, static_cast<int>(x * res_)));
    int idx_y = std::max(0, std::min(ny_ - 1, static_cast<int>(y * res_)));
    
    grid_[idx_y][idx_x] += 0.5f; 
    
    auto phero_msg = std_msgs::msg::Float32();
    phero_msg.data = grid_[idx_y][idx_x];
    phero_pub_->publish(phero_msg);
  }

  void step_pde()
  {
    float dt = 0.1f;
    float h = 1.0f / res_;
    float h2 = h * h;
    
    for (int y = 1; y < ny_ - 1; ++y) {
      for (int x = 1; x < nx_ - 1; ++x) {
        float laplacian = (grid_[y+1][x] + grid_[y-1][x] + grid_[y][x+1] + grid_[y][x-1] - 4.0f * grid_[y][x]) / h2;
        float dC_dt = diffusion_rate_ * laplacian - evaporation_rate_ * grid_[y][x];
        next_grid_[y][x] = std::max(0.0f, grid_[y][x] + dC_dt * dt);
      }
    }
    
    grid_ = next_grid_;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr phero_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  float width_, height_, res_;
  int nx_, ny_;
  float diffusion_rate_, evaporation_rate_;
  
  std::vector<std::vector<float>> grid_;
  std::vector<std::vector<float>> next_grid_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StigmergyNode>());
  rclcpp::shutdown();
  return 0;
}
