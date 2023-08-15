// for test multiple starting and stop camera
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <pthread.hpp>
#include <cstring>
#include <iostream>
#include <sys/prctl.h>


#pragma once
class ThreadManager : public rclcpp::Node
{
public:
ThreadManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
~ThreadManager();
void start_thread(const std::string & thread_name);
void stop_thread(const std::string & thread_name);

void sleep_for(const std::string & tname, int num)
{
	prctl(PR_SET_NAME, tname.c_str(), 0, 0, 0);
	sleep(num);
}

private:
typedef std::unordered_map<std::string, pthread_t> ThreadMap;
ThreadMap tm_;
};