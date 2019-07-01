#pragma once

#include <nodelet/nodelet.h>

#include <memory>

namespace zivid_camera
{
class ZividCamera;
class ZividNodelet : public nodelet::Nodelet
{
private:
  void onInit() override;
  std::unique_ptr<ZividCamera> camera;
};

}  // namespace zivid_camera
