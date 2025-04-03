#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <zivid_rviz_plugin/infield_correction.hpp>

namespace zivid_rviz_plugin
{
InfieldCorrection::InfieldCorrection(QWidget * parent) : rviz_common::Panel(parent)
{
  constexpr int max_content_width = 400;

  layout_ = new QVBoxLayout;

  waiting_for_camera_label_ = new QLabel("Waiting for the Zivid camera node to become available.");
  waiting_for_camera_label_->setVisible(false);
  waiting_for_camera_label_->setWordWrap(true);
  waiting_for_camera_label_->setMaximumWidth(max_content_width);
  layout_->addWidget(waiting_for_camera_label_);

  start_button_ = new QPushButton("Start infield correction");
  start_button_->setMaximumWidth(max_content_width);
  layout_->addWidget(start_button_);
  connect(start_button_, &QPushButton::clicked, this, &InfieldCorrection::onStartClicked);

  capture_button_ = new QPushButton("Capture && Measure");
  capture_button_->setMaximumWidth(max_content_width);
  capture_button_->setVisible(false);
  layout_->addWidget(capture_button_);
  connect(capture_button_, &QPushButton::clicked, this, &InfieldCorrection::onCaptureClicked);

  description_ = new QLabel;
  description_->setWordWrap(true);
  description_->setMaximumWidth(max_content_width);
  layout_->addStretch(1);
  layout_->addWidget(description_);
  layout_->addStretch(1);

  compute_and_write_button_ = new QPushButton("Save correction to camera");
  compute_and_write_button_->setMaximumWidth(max_content_width);
  layout_->addWidget(compute_and_write_button_);
  connect(
    compute_and_write_button_, &QPushButton::clicked, this,
    &InfieldCorrection::onComputeAndWriteClicked);

  auto split_button_container = new QWidget;
  split_button_container->setMaximumWidth(max_content_width);
  layout_->addWidget(split_button_container);

  auto split_button = new QHBoxLayout(split_button_container);
  split_button->setContentsMargins(0, 0, 0, 0);

  remove_last_capture_button_ = new QPushButton("Remove last capture");
  split_button->addWidget(remove_last_capture_button_);
  connect(
    remove_last_capture_button_, &QPushButton::clicked, this,
    &InfieldCorrection::onRemoveLastCaptureClicked);

  restart_captures_button_ = new QPushButton("Restart captures");
  split_button->addWidget(restart_captures_button_);
  connect(
    restart_captures_button_, &QPushButton::clicked, this,
    &InfieldCorrection::onRestartCapturesClicked);

  setLayout(layout_);
}

InfieldCorrection::~InfieldCorrection() = default;

void InfieldCorrection::onInitialize()
{
  node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

  rclcpp::Node::SharedPtr node = node_ptr_->get_raw_node();

  capture_client_ = node->create_client<zivid_interfaces::srv::InfieldCorrectionCapture>(
    "infield_correction/capture");
  remove_last_capture_client_ =
    node->create_client<std_srvs::srv::Trigger>("infield_correction/remove_last_capture");
  start_client_ = node->create_client<std_srvs::srv::Trigger>("infield_correction/start");
  compute_client_ = node->create_client<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute");
  compute_and_write_client_ = node->create_client<zivid_interfaces::srv::InfieldCorrectionCompute>(
    "infield_correction/compute_and_write");

  updateVisibilityAndEnabledStates();

  camera_availability_timer_ =
    node_ptr_->get_raw_node()->create_wall_timer(std::chrono::seconds(3), [this]() {
      const bool is_available_before = !waiting_for_camera_label_->isVisible();
      const bool is_available_after = updateVisibilityAndEnabledStates();
      if (!is_available_before && is_available_after) {
        sendComputeRequestAndShowResult({}, true);
      }
    });

  sendComputeRequestAndShowResult({}, true);

  RCLCPP_INFO(node_ptr_->get_raw_node()->get_logger(), "Infield correction panel initialized");
}

void InfieldCorrection::onCaptureClicked()
{
  if (!updateVisibilityAndEnabledStates()) {
    return;
  }
  RCLCPP_INFO(
    node_ptr_->get_raw_node()->get_logger(), "Infield correction: Sending %s request.",
    capture_client_->get_service_name());
  setState(State::Capturing);
  updateVisibilityAndEnabledStates();
  capture_client_->async_send_request(
    std::make_shared<zivid_interfaces::srv::InfieldCorrectionCapture::Request>(),
    [this](rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCapture>::SharedFuture future) {
      const auto & response = future.get();
      const std::string message =
        (response->success ? "Infield correction capture succeeded."
                           : "Infield correction capture failed: " + response->message);
      setState(State::Started);
      sendComputeRequestAndShowResult(message);
      updateVisibilityAndEnabledStates();
      RCLCPP_INFO(
        node_ptr_->get_raw_node()->get_logger(), "Infield correction: %s", message.c_str());
    });
}

void InfieldCorrection::onComputeAndWriteClicked()
{
  sendComputeRequestAndShowResult("Writing computed infield correction to camera.", false, true);
}

void InfieldCorrection::onRemoveLastCaptureClicked()
{
  if (!updateVisibilityAndEnabledStates()) {
    return;
  }
  RCLCPP_INFO(
    node_ptr_->get_raw_node()->get_logger(), "Infield correction: Sending %s request.",
    remove_last_capture_client_->get_service_name());
  remove_last_capture_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto & response = future.get();
      std::string message;
      if (response->success) {
        message = "Last capture successfully removed.";
      } else {
        message = "Failed to remove last capture: " + response->message;
      }
      sendComputeRequestAndShowResult(message, true);
      RCLCPP_INFO(
        node_ptr_->get_raw_node()->get_logger(), "Infield correction: %s", message.c_str());
    });
}

void InfieldCorrection::onStartClicked()
{
  if (!updateVisibilityAndEnabledStates()) {
    return;
  }
  sendStartRequest("start");
}

void InfieldCorrection::onRestartCapturesClicked()
{
  if (!updateVisibilityAndEnabledStates()) {
    return;
  }
  sendStartRequest("restart");
}

void InfieldCorrection::setState(State state)
{
  if (state != state_) {
    state_ = state;
    start_button_->setVisible(state == State::Uninitialized);
    capture_button_->setVisible(state != State::Uninitialized);
    updateVisibilityAndEnabledStates();
  }
}

void InfieldCorrection::updateStarted(bool started)
{
  if (started && state_ == State::Uninitialized) {
    setState(State::Started);
  } else if (!started && (state_ == State::Started || state_ == State::Capturing)) {
    setState(State::Uninitialized);
  }
}

void InfieldCorrection::sendStartRequest(const std::string & tool_name)
{
  RCLCPP_INFO(
    node_ptr_->get_raw_node()->get_logger(), "Infield correction: Sending %s request.",
    start_client_->get_service_name());
  start_client_->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>(),
    [this, tool_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto & response = future.get();
      std::string message;
      if (response->success) {
        message = "Infield correction tool " + tool_name + "ed, any existing captures erased.";
        setState(State::Started);
      } else {
        message = "Failed to " + tool_name + " infield correction tool: " + response->message;
      }
      sendComputeRequestAndShowResult(message, true);
      updateVisibilityAndEnabledStates();
      RCLCPP_INFO(
        node_ptr_->get_raw_node()->get_logger(), "Infield correction: %s", message.c_str());
    });
}

void InfieldCorrection::sendComputeRequestAndShowResult(
  const std::string & description_preamble, bool silence_zero_captures_error, bool write_to_camera)
{
  auto & client = (write_to_camera ? compute_and_write_client_ : compute_client_);
  RCLCPP_INFO(
    node_ptr_->get_raw_node()->get_logger(), "Infield correction: Sending %s request.",
    client->get_service_name());
  client->async_send_request(
    std::make_shared<zivid_interfaces::srv::InfieldCorrectionCompute::Request>(),
    [this, description_preamble, silence_zero_captures_error, write_to_camera](
      rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCompute>::SharedFuture future) {
      std::string message;
      if (!description_preamble.empty()) {
        message = description_preamble + "\n\n";
      }
      const auto & response = future.get();
      updateStarted(response->infield_correction_started);

      if (silence_zero_captures_error && !response->success && response->number_of_captures == 0) {
        if (state_ == State::Uninitialized) {
          message += "Infield correction tool ready to be started.";
        } else {
          message +=
            "No captures done. We recommend multiple captures with different board positions.";
        }
      } else if (!response->success) {
        message += "Failed to compute infield correction: " + response->message;
      } else {
        if (write_to_camera) {
          setState(State::WriteCompleted);
        }
        message += response->message;
      }

      description_->setText(QString(message.c_str()));
    });
}

bool InfieldCorrection::updateVisibilityAndEnabledStates()
{
  const bool service_available = (start_client_ && start_client_->service_is_ready());
  if (service_available == waiting_for_camera_label_->isVisible()) {
    waiting_for_camera_label_->setVisible(!service_available);
  }

  description_->setEnabled(service_available);
  start_button_->setEnabled(service_available);
  capture_button_->setEnabled(service_available && state_ == State::Started);
  compute_and_write_button_->setEnabled(service_available && state_ == State::Started);
  remove_last_capture_button_->setEnabled(service_available && state_ == State::Started);
  restart_captures_button_->setEnabled(
    service_available && (state_ == State::Started || state_ == State::WriteCompleted));

  return service_available;
}
}  // namespace zivid_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(zivid_rviz_plugin::InfieldCorrection, rviz_common::Panel)
