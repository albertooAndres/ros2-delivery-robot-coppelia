// traffic_light_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TrafficLight : public rclcpp::Node
{
public:
  TrafficLight() : Node("traffic_light")
  {
    declare_parameter<bool>("green", true);          // arranca en verde
    pub_ = create_publisher<std_msgs::msg::Bool>("/crossing_green", 10);

    /* Publica cada 100 ms (10 Hz) */
    timer_ = create_wall_timer(100ms,
        std::bind(&TrafficLight::timerCb, this));

    RCLCPP_INFO(get_logger(),
      "Traffic-light listo. Cambia con:\n"
      "  ros2 param set /traffic_light green true|false");
  }

private:
  void timerCb()
  {
    std_msgs::msg::Bool msg;
    msg.data = get_parameter("green").as_bool();
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/* ---------- main --------------------------------------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrafficLight>());
  rclcpp::shutdown();
  return 0;
}

