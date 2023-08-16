// for test multiple starting and stop camera

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <cstring>
#include <iostream>
#include <sys/prctl.h>
#include "astra_camera/ob_camera_node_factory.h"
#include "astra_camera/device_listener.h"


namespace astra_camera{
class ThreadManager : public rclcpp::Node
{
public:
ThreadManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
~ThreadManager();

void init();
void loop();
void start_thread(const std::string & thread_name);
void stop_thread(const std::string & thread_name);

void work_for(const std::string & tname, int num);


private:
typedef std::unordered_map<std::string, pthread_t> ThreadMap;
ThreadMap tm_;
int loop_number;
int devices_connected_number = 0;
bool success = false;
int success_count = 0;
};
} // end of namespace astra_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::ThreadManager)