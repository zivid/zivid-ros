#pragma once

#ifdef __clang__
#pragma clang system_header
#endif
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC system_header
#endif

#include <zivid_camera/SettingsAcquisitionConfig.h>
#include <zivid_camera/SettingsConfig.h>
#include <zivid_camera/Settings2DAcquisitionConfig.h>
#include <zivid_camera/Settings2DConfig.h>
#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureAndSaveFrame.h>
#include <zivid_camera/Capture2D.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <zivid_camera/CameraInfoModelName.h>
#include <zivid_camera/CameraInfoSerialNumber.h>
#include <zivid_camera/IsConnected.h>
#include <zivid_camera/LoadSettingsFromFile.h>
#include <zivid_camera/LoadSettings2DFromFile.h>
