#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{
constexpr double default_rate_hz = 30.0;
constexpr double default_wait_for_service_timeout_s = 0.0;
}

class Capture2DTrigger final : public rclcpp::Node
{
public:
  Capture2DTrigger() : Node("capture_2d_trigger"), start_time_(now())
  {
    rate_hz_ = declare_parameter<double>("rate_hz", default_rate_hz);
    wait_for_service_timeout_s_ =
      declare_parameter<double>("wait_for_service_timeout_s", default_wait_for_service_timeout_s);
    if (rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Parameter rate_hz must be positive. Falling back to default %.1f Hz.", default_rate_hz);
      rate_hz_ = default_rate_hz;
    }
    if (wait_for_service_timeout_s_ < 0.0)
    {
      RCLCPP_WARN(
        get_logger(),
        "Parameter wait_for_service_timeout_s must be >= 0. Falling back to default %.1f s.",
        default_wait_for_service_timeout_s);
      wait_for_service_timeout_s_ = default_wait_for_service_timeout_s;
    }

    capture_2d_client_ = create_client<std_srvs::srv::Trigger>("capture_2d");
  }

  bool waitForServiceAndStartTimer()
  {
    while (rclcpp::ok())
    {
      if (capture_2d_client_->wait_for_service(std::chrono::seconds(1)))
      {
        break;
      }

      if (!waiting_for_service_.exchange(true))
      {
        RCLCPP_INFO(get_logger(), "Waiting for capture_2d service to become available...");
      }

      if (wait_for_service_timeout_s_ > 0.0)
      {
        const auto elapsed = now() - start_time_;
        if (elapsed.seconds() >= wait_for_service_timeout_s_)
        {
          RCLCPP_ERROR(
            get_logger(),
            "capture_2d service was not available after %.1f s. Shutting down.",
            wait_for_service_timeout_s_);
          rclcpp::shutdown();
          return false;
        }
      }
    }

    if (!rclcpp::ok())
    {
      return false;
    }

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&Capture2DTrigger::onTimer, this));
    return true;
  }

private:
  void onTimer()
  {
    if (!capture_2d_client_->service_is_ready())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "capture_2d service not ready; skipping tick");
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
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "capture_2d failed: %s",
            response->message.c_str());
        }
      });
  }

  double rate_hz_{default_rate_hz};
  double wait_for_service_timeout_s_{default_wait_for_service_timeout_s};
  std::atomic_bool request_in_flight_{false};
  std::atomic_bool waiting_for_service_{false};
  rclcpp::Time start_time_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_2d_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Capture2DTrigger>();
  if (node->waitForServiceAndStartTimer())
  {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
