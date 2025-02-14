#include <chrono>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>

/*
 * This sample shows how to perform infield correction using services from `zivid_camera`.
 */

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

auto create_trigger_client(std::shared_ptr<rclcpp::Node> & node, const std::string & name)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(name);
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for the " + name + " service to appear...");
  }
  RCLCPP_INFO_STREAM(node->get_logger(), name + " service is available");
  return client;
}

void create_infield_correction_compute_client(const std::shared_ptr<rclcpp::Node> & node)
{
  using zivid_interfaces::srv::InfieldCorrectionCompute;

  auto client = node->create_client<InfieldCorrectionCompute>("infield_correction/compute");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for the infield_correction/compute service to appear...");
  }

  const std::string filename = "zivid_sample_capture_and_save.zdf";

  RCLCPP_INFO(node->get_logger(), "Sending infield correction compute request");
  auto result = client->async_send_request(std::make_shared<InfieldCorrectionCompute::Request>());
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the infield correction compute service");
  }

  auto capture_response = result.get();
  if (capture_response->success) {
    RCLCPP_INFO(
      node->get_logger(),
      "Infield correction computed successfully. Dimension accuracy: %g. z_min: %g. z_max: %g. "
      "Message: %s",
      capture_response->dimension_accuracy, capture_response->z_min, capture_response->z_max,
      capture_response->message.c_str());
  } else {
    RCLCPP_INFO(
      node->get_logger(), "Infield correction compute failed: %s",
      capture_response->message.c_str());
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sample_infield_correction");
  RCLCPP_INFO(node->get_logger(), "Started the sample_infield_correction node");

  auto restart_client = create_trigger_client(node, "infield_correction/restart_captures");
  RCLCPP_INFO(node->get_logger(), "Triggering infield_correction/restart");
  auto restart_future =
    restart_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (
    rclcpp::spin_until_future_complete(node, restart_future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to send infield correction restart request");
  }
  if (auto restart_result = restart_future.get(); !restart_result->success) {
    fatal_error(
      node->get_logger(), "Failed to restart infield correction: " + restart_result->message);
  }

  auto capture_client = create_trigger_client(node, "infield_correction/capture");
  auto trigger_capture = [&]() -> bool {
    RCLCPP_INFO(node->get_logger(), "Triggering infield correction capture");
    auto future =
      capture_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
      fatal_error(node->get_logger(), "Failed to send capture request");
    }
    auto result = future.get();
    RCLCPP_INFO_STREAM(
      node->get_logger(), (result->success ? "Infield calibration capture success: "
                                           : "Failed to capture for infield correction: ") +
                            result->message);
    return result->success;
  };

  constexpr int num_successful_captures_target = 5;
  for (int num_successful_captures = 0; num_successful_captures < num_successful_captures_target;) {
    if (num_successful_captures > 0) {
      constexpr int wait_seconds = 5;
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Waiting for " + std::to_string(wait_seconds) + " seconds before taking the next capture.");
      rclcpp::spin_until_future_complete(
        node, std::promise<bool>().get_future(), std::chrono::seconds(wait_seconds));
    }

    if (trigger_capture()) {
      num_successful_captures += 1;
    }
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Finished gathering " + std::to_string(num_successful_captures_target) +
                          " successful captures, proceeding to compute infield correction.");

  create_infield_correction_compute_client(node);

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
