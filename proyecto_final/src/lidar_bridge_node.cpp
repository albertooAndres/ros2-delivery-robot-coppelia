// src/lidar_bridge_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <algorithm>
#include <limits>
#include <chrono>          

class LidarBridge : public rclcpp::Node
{
public:
  LidarBridge() : Node("lidar_bridge")
  {
    declare_parameter<double>("safe_distance", 0.17);   // m
    declare_parameter<double>("check_hz",      20.0);   // Hz del timer
    rclcpp::QoS qos(10);  qos.best_effort();

    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&LidarBridge::scanCb, this, std::placeholders::_1));

    cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety_stop", 10);

    const double period_ms = 1000.0 / get_parameter("check_hz").as_double();
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(period_ms)),
        std::bind(&LidarBridge::timerCb, this));

    RCLCPP_INFO(get_logger(),
        "lidar_bridge activo: frena si obstáculo < %.2fm",
        get_parameter("safe_distance").as_double());
  }

private:
  // ───────────── callbacks ────────────────────────────────────────────────
  /*
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto &r = msg->ranges;
    min_dist_ = *std::min_element(r.begin(), r.end());
  }*/
  
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
	  double best = std::numeric_limits<double>::infinity();

	  for (float d : msg->ranges)
	  {
	    if (std::isfinite(d))            // descarta inf y NaN
	      best = std::min(best, static_cast<double>(d));
	  }
	  min_dist_ = best;                  

   }

  //------------------------------------------------------------------
 
  void timerCb()
  {
    const double safe = get_parameter("safe_distance").as_double();

    if (min_dist_ < safe && !stopped_)            // obstáculo → STOP
    {
      geometry_msgs::msg::Twist stop;             // todo a 0
      cmd_pub_->publish(stop);
      stopped_ = true;
      RCLCPP_WARN(get_logger(),
                  "Obstáculo a %.2fm → STOP", min_dist_);
    }
    else if (min_dist_ >= safe && stopped_)       // libre otra vez
    {
      stopped_ = false;
      RCLCPP_INFO(get_logger(), "Camino libre de nuevo");
      // El seguidor de línea retomará con su próximo /cmd_vel
    }
    std_msgs::msg::Bool flag;
    flag.data = stopped_;
    stop_pub_->publish(flag);
  }

  // ───────────── miembros ────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            stop_pub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;

  double min_dist_{std::numeric_limits<double>::infinity()};
  bool   stopped_{false};
};

// ─────────────────────────── main ─────────────────────────────────────────
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarBridge>());
  rclcpp::shutdown();
  return 0;
}

