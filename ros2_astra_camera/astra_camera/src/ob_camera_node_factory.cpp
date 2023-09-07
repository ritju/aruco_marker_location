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

#include <filesystem>
#include <fcntl.h>
#include "astra_camera/ob_camera_node_factory.h"

namespace astra_camera {
OBCameraNodeFactory::OBCameraNodeFactory(const rclcpp::NodeOptions& node_options)
    : Node("astra_camera_node", "/", node_options), logger_(get_logger()) {
  // init();
  thread_check_ = std::thread([&](){checkConnection();});
  thread_check_.detach();
}

OBCameraNodeFactory::OBCameraNodeFactory(const std::string& node_name, const std::string& ns,
                                         const rclcpp::NodeOptions& node_options)
    : Node(node_name, ns, node_options), logger_(get_logger()) {
  init();
  // thread_check_ = std::thread(std::bind(&OBCameraNodeFactory::checkConnection, this));
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  sem_unlink(DEFAULT_SEM_NAME.c_str());
  openni::OpenNI::shutdown();
}

void OBCameraNodeFactory::init() {
  if (std::filesystem::exists("/dev/shm/sem." + DEFAULT_SEM_NAME)) {
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
  auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Initialize failed\n%s\n", openni::OpenNI::getExtendedError());
    exit(-1);
  }
  parameters_ = std::make_shared<Parameters>(this);

  use_uvc_camera_ = declare_parameter_if_not_declared<bool>("uvc_camera.enable", false);
  serial_number_ = declare_parameter_if_not_declared<std::string>("serial_number", "");
  number_of_devices_ = declare_parameter_if_not_declared<int>("number_of_devices", 1);
  reconnection_delay_ = declare_parameter_if_not_declared<int>("reconnection_delay", 1);
  depth_try_number_ = declare_parameter_if_not_declared<int>("depth_try_number", 1);
  depth_reconnection_delay_ = declare_parameter_if_not_declared<int>("depth_reconnection_delay", 1);

  auto connected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceConnected(device_info);
  };
  auto disconnected_cb = [this](const openni::DeviceInfo* device_info) {
    onDeviceDisconnected(device_info);
  };
  if(device_listener_)
  {
    device_listener_.release();
    RCLCPP_INFO(logger_, "====device_listener release");
  }
  
  device_listener_ = std::make_unique<DeviceListener>(connected_cb, disconnected_cb);
  if(device_listener_)
  {
    RCLCPP_INFO(logger_, "****device_listener exists.");
  }
  using namespace std::chrono_literals;
  connecting_ = false;
  check_connection_timer_ = this->create_wall_timer(1s, [this] { checkConnectionTimer(); });
}

template <typename T>
T OBCameraNodeFactory::declare_parameter_if_not_declared(const std::string& name, T default_value)
{
  T return_value;
  if(!this->has_parameter(name))
  {
    return_value = this->declare_parameter(name, default_value);
  }
  else
  {
    return_value = this->get_parameter(name).get_value<T>();
  }
  return return_value;
}

void OBCameraNodeFactory::startDevice() {
  RCLCPP_INFO_STREAM(logger_, "starting device " << serial_number_);
  if (ob_camera_node_) {
    RCLCPP_INFO(logger_, "has ob_camera_node_");
    // ob_camera_node_.reset();
    // ob_camera_node_ = NULL;
  }
  RCLCPP_INFO(logger_, "check device_");
  CHECK_NOTNULL(device_);
  RCLCPP_INFO(logger_, "check parameters_");
  CHECK_NOTNULL(parameters_);
  if (use_uvc_camera_) {
    RCLCPP_INFO(logger_, "use uvc_camera_driver_");
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
      RCLCPP_INFO(logger_, "reset uvc_camera_driver_");
    }
    RCLCPP_INFO(logger_, "assign uvc_camera_driver_");
    uvc_camera_driver_ = std::make_shared<UVCCameraDriver>(this, parameters_, serial_number_);
    RCLCPP_INFO(logger_, "assign ob_camera_node_");
    auto ptr_old = ob_camera_node_.release();
    // delete ptr_old;
    ob_camera_node_ =
        std::make_unique<OBCameraNode>(this, device_, parameters_, uvc_camera_driver_);
  } else {
    RCLCPP_INFO(logger_, "not use uvc_camera_driver_");
    ob_camera_node_ = std::make_unique<OBCameraNode>(this, device_, parameters_);
  }
  device_connected_ = true;
  if (is_first_connection_) {
    is_first_connection_ = false;
    RCLCPP_INFO_STREAM(logger_, "first connection");
  }
  RCLCPP_INFO(logger_, "start_device end.");
}

void OBCameraNodeFactory::onDeviceConnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName());
  if (device_info->getUri() == nullptr) {
    RCLCPP_ERROR_STREAM(logger_, "Device connected: " << device_info->getName() << " uri is null");
    return;
  }
  auto device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
  if (device_sem == (void*)SEM_FAILED) {
    RCLCPP_ERROR(logger_, "Failed to create semaphore");
    return;
  }
  RCLCPP_INFO_STREAM(logger_, "Waiting for device to be ready");
  int ret = sem_wait(device_sem);
  if (!ret && !connected_devices_.count(device_info->getUri())) {
    auto device = std::make_shared<openni::Device>();
    RCLCPP_INFO_STREAM(logger_, "Trying to open device: " << device_info->getUri());
    int cur_number = 0;
    openni::Status rc;
    do {
      rc = device->open(device_info->getUri());
      cur_number++;
      if (rc == openni::STATUS_OK)
        break;
      usleep(depth_reconnection_delay_);
    } while(cur_number <= depth_try_number_);
    RCLCPP_INFO(logger_, "try %d times to open device successfully.", cur_number);
    if (rc != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open device: " << device_info->getUri() << " error: "
                                                             << openni::OpenNI::getExtendedError());
      RCLCPP_INFO(logger_, "openni::STATUS: %X", rc); 
      RCLCPP_INFO(logger_, "errno: %d", errno);
      if (errno == EBUSY) {
        RCLCPP_ERROR_STREAM(logger_, "Device is already opened OR device is in use");
        connected_devices_[device_info->getUri()] = *device_info;
      }
    } else {
      char serial_number[64];
      int data_size = sizeof(serial_number);
      rc = device->getProperty(openni::OBEXTENSION_ID_SERIALNUMBER, serial_number, &data_size);
      if (rc != openni::STATUS_OK) {
        RCLCPP_ERROR_STREAM(logger_,
                            "Failed to get serial number: " << openni::OpenNI::getExtendedError());
      } else if (serial_number_.empty() || serial_number == serial_number_) {
        serial_uri.clear();
        serial_number_ =  serial_number;
        serial_uri.insert({serial_number_, std::string(device_info->getUri())});
        RCLCPP_INFO_STREAM(logger_, "Device connected: " << device_info->getName()
                                                         << " serial number: " << serial_number);
        device_uri_ = device_info->getUri();
        connected_devices_[device_uri_] = *device_info;
        device_ = device;
        if (!is_first_connection_) {
          std::this_thread::sleep_for(std::chrono::seconds(reconnection_delay_));
        }
        startDevice();
      }
      else
      {
        RCLCPP_INFO(logger_, "dst serialnumber: %s", serial_number_.c_str());
        RCLCPP_INFO(logger_, "cur serialnumber: %s", serial_number);
      }
    }
    if (!device_connected_) {
      RCLCPP_INFO(logger_, "close device.");
      device->close();
    }
  }
  RCLCPP_INFO_STREAM(logger_, "Release device semaphore");
  sem_post(device_sem);
  RCLCPP_INFO_STREAM(logger_, "Release device semaphore done");
  if (connected_devices_.size() == number_of_devices_) {
    RCLCPP_INFO_STREAM(logger_, "All devices connected");
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeFactory::onDeviceDisconnected(const openni::DeviceInfo* device_info) {
  RCLCPP_INFO(logger_, "onDeviceDisconnected callback.");
  if (device_uri_ == device_info->getUri()) {
    device_uri_.clear();
    if (ob_camera_node_) {
      ob_camera_node_.reset();
    }
    if (device_) {
      device_->close();
      device_.reset();
    }
    if (uvc_camera_driver_) {
      uvc_camera_driver_.reset();
    }
    RCLCPP_INFO_STREAM(logger_, "Device disconnected: " << device_info->getUri());
    device_connected_ = false;
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeFactory::checkConnectionTimer() {
  if (!device_connected_) {
    RCLCPP_INFO_STREAM(logger_, "wait for device connect... ");
  }
}

void OBCameraNodeFactory::checkConnection() {
  while(true)
  {
    sleep(3);
    // openni::OpenNI::initialize();
    openni::Array<openni::DeviceInfo> device_info_list;
    openni::OpenNI::enumerateDevices(&device_info_list);
    
    std::string bus_code;
    size_t pose_start = device_uri_.find_last_of('/');
    bus_code = device_uri_.substr(pose_start + 1);
    for(size_t i = bus_code.size(); i < 3; i++)
    {
      bus_code = "0" + bus_code;
    }
    RCLCPP_INFO(logger_, "bus_code: %s", bus_code.c_str());

    if(!std::filesystem::exists("/dev/bus/usb/001/"+bus_code))
    {
      connecting_ = true;
      RCLCPP_INFO_STREAM(logger_, "off line number: " << ++number_off_ << "\n");
      if (std::filesystem::exists("/dev/shm/sem." + DEFAULT_SEM_NAME)) {
        sem_unlink(DEFAULT_SEM_NAME.c_str());
      }
      if(device_)
      {
        device_->close();
        device_.reset();
      }
      init();
      RCLCPP_INFO_STREAM(logger_, "-------------------\n");
    }
    else
    {       
      RCLCPP_INFO_STREAM(logger_, "ok ");
      RCLCPP_INFO_STREAM(logger_, "\n");
    }
    if(!connecting_)
    {
      // openni::OpenNI::shutdown();
    }
  }
}



}  // namespace astra_camera
