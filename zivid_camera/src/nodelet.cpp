#include "nodelet.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <exception>
#include <memory>

#include "zivid_camera.h"

namespace zivid_camera
{
void ZividNodelet::onInit()
{
  try {
    // Important: use non-multi-threaded callback queues (getNodeHandle and getPrivateNodeHandle).
    camera = std::make_unique<ZividCamera>(getNodeHandle(), getPrivateNodeHandle());
  } catch (const std::exception & e) {
    NODELET_ERROR_STREAM("Failed to initialize camera driver. Exception: \"" << e.what() << "\"");
    throw;
  } catch (...) {
    NODELET_ERROR_STREAM("Failed to initialize camera driver (unknown exception)");
    throw;
  }
}
}  // namespace zivid_camera

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wglobal-constructors"
#endif

PLUGINLIB_EXPORT_CLASS(zivid_camera::ZividNodelet, nodelet::Nodelet)

#ifdef __clang__
#pragma clang diagnostic pop
#endif
