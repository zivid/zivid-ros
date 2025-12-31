#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr double default_rate_hz = 30.0;
}

class Capture2DTrigger final : public rclcpp::Node
{
public:
  Capture2DTrigger() : Node("capture_2d_trigger")
  {
    rate_hz_ = declare_parameter<double>("rate_hz", default_rate_hz);
    if (rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Parameter rate_hz must be positive. Falling back to default %.1f Hz.", default_rate_hz);
      rate_hz_ = default_rate_hz;
    }

    capture_2d_client_ = create_client<std_srvs::srv::Trigger>("capture_2d");

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(period, std::bind(&Capture2DTrigger::onTimer, this));
  }

private:
  void onTimer()
  {
    if (!capture_2d_client_->service_is_ready())
    {
      if (!waiting_for_service_.exchange(true))
      {
        RCLCPP_INFO(get_logger(), "Waiting for capture_2d service to become available...");
      }
      return;
    }

    waiting_for_service_.store(false);

    if (request_in_flight_.exchange(true))
    {
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    capture_2d_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
      {
        request_in_flight_.store(false);
        const auto response = future.get();
        if (!response->success)
        {
          RCLCPP_WARN(get_logger(), "capture_2d failed: %s", response->message.c_str());
        }
      });
  }

  double rate_hz_{default_rate_hz};
  std::atomic_bool request_in_flight_{false};
  std::atomic_bool waiting_for_service_{false};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_2d_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Capture2DTrigger>());
  rclcpp::shutdown();
  return 0;
}
