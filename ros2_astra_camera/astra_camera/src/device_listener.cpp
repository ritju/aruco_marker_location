/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2022 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include "astra_camera/device_listener.h"
#include <magic_enum.hpp>

namespace astra_camera {

DeviceListener::DeviceListener(DeviceConnectedCb connected_cb, DeviceDisconnectedCb disconnected_cb)
    : openni::OpenNI::DeviceConnectedListener(),
      openni::OpenNI::DeviceDisconnectedListener(),
      openni::OpenNI::DeviceStateChangedListener(),
      logger_(rclcpp::get_logger("device_listener")),
      connected_cb_(std::move(connected_cb)),
      disconnected_cb_(std::move(disconnected_cb)) {
  openni::Status rc;
  RCLCPP_INFO(logger_, "device listener constructor.");
  openni::OpenNI::shutdown();
  rc = openni::OpenNI::initialize();
  if(rc == openni::STATUS_OK)
  {
    RCLCPP_INFO_STREAM(logger_, "initialize() success. status: " << magic_enum::enum_name(rc));
  }
  else
  {
    RCLCPP_INFO_STREAM(logger_, "initialize() fail. status: " << magic_enum::enum_name(rc));
  }
  rc = openni::OpenNI::addDeviceConnectedListener(this);
  if(rc == openni::STATUS_OK)
  {
    RCLCPP_INFO(logger_, "add device connected listener success.");
  }
  else
  {
    RCLCPP_INFO(logger_, "add device connected listener fail.");
  }
  rc = openni::OpenNI::addDeviceDisconnectedListener(this);
  if(rc == openni::STATUS_OK)
  {
    RCLCPP_INFO(logger_, "add device dis_connected listener success.");
  }
  else
  {
    RCLCPP_INFO(logger_, "add device dis_connected listener fail.");
  }
  rc = openni::OpenNI::addDeviceStateChangedListener(this);
  if(rc == openni::STATUS_OK)
  {
    RCLCPP_INFO(logger_, "add device status changed listener success.");
  }
  else
  {
    RCLCPP_INFO(logger_, "add device status changed listener fail.");
  }
  // get list of currently connected devices
  openni::Array<openni::DeviceInfo> device_info_list;
  openni::OpenNI::enumerateDevices(&device_info_list);
  RCLCPP_INFO(logger_, "device number: %d", device_info_list.getSize());
  for (int i = 0; i < device_info_list.getSize(); ++i) {
    RCLCPP_INFO(logger_, "device uri: %s", device_info_list[i].getUri());
    onDeviceConnected(&device_info_list[i]);
  }
}

DeviceListener::~DeviceListener() {
  openni::OpenNI::removeDeviceConnectedListener(this);
  openni::OpenNI::removeDeviceDisconnectedListener(this);
  openni::OpenNI::removeDeviceStateChangedListener(this);
}

void DeviceListener::onDeviceStateChanged(const openni::DeviceInfo* device_info,
                                          openni::DeviceState state) {
  RCLCPP_INFO_STREAM(logger_, "Device " << device_info->getUri() << " state changed to "
                                        << magic_enum::enum_name(state));
}

void DeviceListener::onDeviceConnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "onDeviceConnected");

  connected_cb_(device_info);
}

void DeviceListener::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "onDeviceDisconnected");
  disconnected_cb_(device_info);
}

}  // namespace astra_camera
