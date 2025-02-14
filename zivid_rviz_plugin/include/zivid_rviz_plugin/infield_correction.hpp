#ifndef RVIZ_INFIELD_CORRECTION_HPP
#define RVIZ_INFIELD_CORRECTION_HPP

#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <zivid_interfaces/srv/infield_correction_capture.hpp>
#include <zivid_interfaces/srv/infield_correction_compute.hpp>

namespace zivid_rviz_plugin
{

class InfieldCorrection : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit InfieldCorrection(QWidget * parent = nullptr);
  ~InfieldCorrection() override;

  void onInitialize() override;

private Q_SLOTS:
  void onCaptureClicked();
  void onComputeAndWriteClicked();
  void onRemoveLastCaptureClicked();
  void onStartClicked();
  void onRestartCapturesClicked();

private:
  enum class State
  {
    Uninitialized,
    Started,
    Capturing,
    WriteCompleted,
  };

  void setState(State state);
  void updateStarted(bool started);
  void sendStartRequest(const std::string & tool_name);
  void sendComputeRequestAndShowResult(
    const std::string & description_preamble = {}, bool silence_zero_captures_error = false,
    bool write_to_camera = false);
  bool updateVisibilityAndEnabledStates();

  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;

  rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCapture>::SharedPtr capture_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr remove_last_capture_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCompute>::SharedPtr compute_client_;
  rclcpp::Client<zivid_interfaces::srv::InfieldCorrectionCompute>::SharedPtr
    compute_and_write_client_;

  rclcpp::TimerBase::SharedPtr camera_availability_timer_;

  State state_ = State::Uninitialized;
  QLabel * waiting_for_camera_label_ = nullptr;
  QLabel * description_ = nullptr;
  QVBoxLayout * layout_ = nullptr;
  QPushButton * start_button_ = nullptr;
  QPushButton * capture_button_ = nullptr;
  QPushButton * compute_and_write_button_ = nullptr;
  QPushButton * remove_last_capture_button_ = nullptr;
  QPushButton * restart_captures_button_ = nullptr;
};

}  // namespace zivid_rviz_plugin

#endif  //RVIZ_INFIELD_CORRECTION_HPP
