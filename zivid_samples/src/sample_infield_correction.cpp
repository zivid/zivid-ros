#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>
#include <zivid_interfaces/srv/infield_correction_verify.hpp>

static const auto read_only_parameter =
  rcl_interfaces::msg::ParameterDescriptor{}.set__read_only(true);

void fatal_error(const rclcpp::Logger & logger, const std::string & message)
{
  RCLCPP_ERROR_STREAM(logger, message);
  throw std::runtime_error(message);
}

template <typename List, typename ToStringFunc>
std::string join_list_to_string(const List & list, ToStringFunc && to_string_func)
{
  return list.empty()
           ? std::string()
           : std::accumulate(
               std::next(std::begin(list)), std::end(list), to_string_func(*std::begin(list)),
               [&](const std::string & str, const auto & entry) {
                 return str + ", " + to_string_func(entry);
               });
}

template <typename Enum>
Enum get_parameter_enum(
  rclcpp::Node & node, const std::string & name, const std::map<std::string, Enum> & name_value_map)
{
  const auto value = node.declare_parameter<std::string>(name, "", read_only_parameter);
  const auto it = name_value_map.find(value);
  if (it == name_value_map.end()) {
    const std::string valid_values =
      join_list_to_string(name_value_map, [](auto && pair) { return pair.first; });
    fatal_error(
      node.get_logger(), "Invalid value for parameter '" + name + "': '" + value +
                           "'. Expected one of: " + valid_values + ".");
  }
  RCLCPP_INFO(node.get_logger(), "%s", ("Got parameter " + name + ": " + value).c_str());
  return it->second;
}

enum class InfieldCorrectionOperation
{
  Verify,
  Correct,
  CorrectAndWrite,
  Read,
  Reset,
};

const std::map<std::string, InfieldCorrectionOperation> infield_correction_operation_map = {
  {"verify", InfieldCorrectionOperation::Verify},
  {"correct", InfieldCorrectionOperation::Correct},
  {"correct_and_write", InfieldCorrectionOperation::CorrectAndWrite},
  {"read", InfieldCorrectionOperation::Read},
  {"reset", InfieldCorrectionOperation::Reset},
};

std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> create_trigger_client(
  const rclcpp::Node::SharedPtr & node, const std::string & service_name)
{
  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for the %s service to appear...", client->get_service_name());
  }
  return client;
}

std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCompute>>
create_infield_correction_compute_client(
  const rclcpp::Node::SharedPtr & node, const std::string & service_name)
{
  auto client = node->create_client<zivid_interfaces::srv::InfieldCorrectionCompute>(service_name);
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for the %s service to appear...", client->get_service_name());
  }
  return client;
}

std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionVerify>>
create_infield_correction_verify_client(const rclcpp::Node::SharedPtr & node)
{
  auto client = node->create_client<zivid_interfaces::srv::InfieldCorrectionVerify>(
    "infield_correction/verify");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      fatal_error(node->get_logger(), "Client interrupted while waiting for service to appear.");
    }
    RCLCPP_INFO(
      node->get_logger(), "Waiting for the %s service to appear...", client->get_service_name());
  }
  return client;
}

bool request_trigger_and_print_response(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> & client)
{
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the infield correction compute service");
  }
  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Trigger results (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
  return result->success;
}

void request_infield_correction_verify_and_print_response(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionVerify>> & client)
{
  auto future = client->async_send_request(
    std::make_shared<zivid_interfaces::srv::InfieldCorrectionVerify::Request>());
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the infield correction verify service");
  }
  auto result = future.get();
  RCLCPP_INFO(node->get_logger(), "Infield verification results (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(
    node->get_logger(), "  Local dimension trueness: %g", result->local_dimension_trueness);
  RCLCPP_INFO(
    node->get_logger(), "  Position: %g, %g, %g", result->position.x, result->position.y,
    result->position.z);
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
}

void request_infield_correction_compute_and_print_response(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCompute>> & client,
  const std::string & name)
{
  auto future = client->async_send_request(
    std::make_shared<zivid_interfaces::srv::InfieldCorrectionCompute::Request>());
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    fatal_error(node->get_logger(), "Failed to call the infield correction " + name + " service");
  }
  auto result = future.get();
  RCLCPP_INFO(
    node->get_logger(), "Infield correction compute results (%s):", client->get_service_name());
  RCLCPP_INFO(node->get_logger(), "  Success: %s", result->success ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "  Number of captures: %d", result->number_of_captures);
  RCLCPP_INFO(node->get_logger(), "  Current trueness error: %g", result->current_trueness_error);
  RCLCPP_INFO(node->get_logger(), "  Average trueness error: %g", result->average_trueness_error);
  RCLCPP_INFO(node->get_logger(), "  Maximum trueness error: %g", result->maximum_trueness_error);
  RCLCPP_INFO(node->get_logger(), "  Dimension accuracy: %g", result->dimension_accuracy);
  RCLCPP_INFO(node->get_logger(), "  Z min: %g", result->z_min);
  RCLCPP_INFO(node->get_logger(), "  Z max: %g", result->z_max);
  RCLCPP_INFO(
    node->get_logger(), "  Message: %s",
    result->message.empty() ? "" : (R"(""")" + result->message + R"(""")").c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sample_infield_correction");

  const InfieldCorrectionOperation operation =
    get_parameter_enum(*node, "operation", infield_correction_operation_map);

  switch (operation) {
    case InfieldCorrectionOperation::Verify: {
      RCLCPP_INFO(node->get_logger(), "--- Starting infield correction: Verify ---");
      auto verify_client = create_infield_correction_verify_client(node);
      request_infield_correction_verify_and_print_response(node, verify_client);
    } break;
    case InfieldCorrectionOperation::Correct:
    case InfieldCorrectionOperation::CorrectAndWrite: {
      RCLCPP_INFO(node->get_logger(), "--- Starting infield correction ---");
      auto start_client = create_trigger_client(node, "infield_correction/start");
      if (!request_trigger_and_print_response(node, start_client)) {
        fatal_error(node->get_logger(), "Could not start infield correction, aborting.");
      }

      auto capture_client = create_trigger_client(node, "infield_correction/capture");
      auto compute_client =
        create_infield_correction_compute_client(node, "infield_correction/compute");

      constexpr int wait_seconds = 5;
      constexpr int num_successful_captures_target = 3;
      RCLCPP_INFO_STREAM(
        node->get_logger(), "--- Starting captures, will proceed until " +
                              std::to_string(num_successful_captures_target) +
                              " captures have completed with a detected calibration board ---");

      bool sleep_before_capture = false;
      for (int num_successful_captures = 0;
           num_successful_captures < num_successful_captures_target;) {
        if (sleep_before_capture) {
          RCLCPP_INFO_STREAM(
            node->get_logger(), "--- Waiting for " + std::to_string(wait_seconds) +
                                  " seconds before taking the next capture ---");
          rclcpp::spin_until_future_complete(
            node, std::promise<bool>().get_future(), std::chrono::seconds(wait_seconds));
        } else {
          sleep_before_capture = true;
        }

        const bool success = request_trigger_and_print_response(node, capture_client);
        if (success) {
          num_successful_captures += 1;
          // Run compute on the gathered captures so far. This provides useful information about the
          // expected effect of the correction with the current captures.
          request_infield_correction_compute_and_print_response(node, compute_client, "compute");
        }
      }

      RCLCPP_INFO(node->get_logger(), "--- Captures complete ---");

      if (operation == InfieldCorrectionOperation::CorrectAndWrite) {
        RCLCPP_INFO_STREAM(node->get_logger(), "--- Writing correction results to camera ---");
        auto compute_and_write_client =
          create_infield_correction_compute_client(node, "infield_correction/compute_and_write");
        request_infield_correction_compute_and_print_response(
          node, compute_and_write_client, "compute and write");
      }

    } break;
    case InfieldCorrectionOperation::Read: {
      RCLCPP_INFO(node->get_logger(), "--- Starting infield correction: Read ---");
      auto read_client = create_trigger_client(node, "infield_correction/read");
      request_trigger_and_print_response(node, read_client);
    } break;
    case InfieldCorrectionOperation::Reset: {
      RCLCPP_INFO(node->get_logger(), "--- Starting infield correction: Reset ---");
      auto reset_client = create_trigger_client(node, "infield_correction/reset");
      request_trigger_and_print_response(node, reset_client);
    } break;
    default: {
      fatal_error(
        node->get_logger(), "Unknown operation: " + std::to_string(static_cast<int>(operation)));
    } break;
  }

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
