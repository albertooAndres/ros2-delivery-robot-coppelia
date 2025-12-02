// ir_line_follower_node.cpp  (control proporcional de línea)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

class IrLineFollower : public rclcpp::Node
{
public:
  IrLineFollower() : Node("ir_line_follower")
  {
    // ─── Parámetros ─────────────────────────────────────────────
    declare_parameter<double>("v_forward",  0.12);   // m/s (v)
    declare_parameter<double>("k_line",     5);    // ganancia K
    declare_parameter<double>("grey_ref",   0.35);   // valor target (negro≈0)
    declare_parameter<double>("b_half",     0.033);   // mitad ejes rueda (m)
    declare_parameter<double>("pub_rate",  20.0);    // Hz

    // ─── Suscriptor a la luminancia ────────────────────────────
    lum_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/ir_line", rclcpp::SensorDataQoS(),
        std::bind(&IrLineFollower::lumCallback, this, std::placeholders::_1));
    
    safety_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/safety_stop", rclcpp::SensorDataQoS(),
        std::bind(&IrLineFollower::safetyCallback, this, std::placeholders::_1));

    // ─── Publicador /cmd_vel ───────────────────────────────────
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // ─── Timer periódico ───────────────────────────────────────
    auto dt = std::chrono::milliseconds(
                static_cast<int>(1000.0 / get_parameter("pub_rate").as_double()));
    timer_ = create_wall_timer(dt, std::bind(&IrLineFollower::timerCb, this));

    RCLCPP_INFO(get_logger(), "IrLineFollower P-control listo");
  }

private:
  //------------------------------------------------------------------
  void lumCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    last_lum_ = msg->data;               // guarda el valor (0-1)
    have_lum_ = true;
  }
  
  void safetyCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    safety_stop_ = msg->data;
  }

  //------------------------------------------------------------------
  void timerCb()
  {
    if (!have_lum_) return;              // espera primera lectura
    
    if (safety_stop_)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());  // Twist(0,0)
      return;
    }

    const double v   = get_parameter("v_forward").as_double();
    const double k   = get_parameter("k_line").as_double();
    const double ref = get_parameter("grey_ref").as_double();

    double omega = -k * (last_lum_ - ref);   // ley de control ω

    // Publica Twist (cmd_vel)
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x  = v;
    cmd.angular.z = omega;
    cmd_pub_->publish(cmd);
  }
  //------------------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lum_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double last_lum_{0.0};
  bool   have_lum_{false};
  bool   safety_stop_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IrLineFollower>());
  rclcpp::shutdown();
  return 0;
}

