// Copyright 2026 Zivid AS
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Zivid AS nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <Zivid/VersionConstants.h>
#if ZIVID_CORE_VERSION_MAJOR > 2 || \
  (ZIVID_CORE_VERSION_MAJOR == 2 && ZIVID_CORE_VERSION_MINOR >= 18)
#define ZIVID_CAMERA_HAS_CAMERA_HEALTH
#endif

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <zivid_interfaces/msg/camera_health.hpp>

#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
#include <Zivid/CameraHealth.h>
#endif

// Conversions used by CameraHealthController. They are not part of the driver's API, and are only
// declared in a header so that they can be tested without a camera.
namespace zivid_camera::detail
{
#ifdef ZIVID_CAMERA_HAS_CAMERA_HEALTH
zivid_interfaces::msg::CameraHealth toRosCameraHealth(const Zivid::CameraHealth & health);
#endif

// Sets the level, message and values of a diagnostic status from a camera health message. The name
// and hardware ID are left unchanged.
void fillDiagnosticStatus(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const zivid_interfaces::msg::CameraHealth & health);
}  // namespace zivid_camera::detail
