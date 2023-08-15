#include "astra_camera/test/thread_manager.hpp"

ThreadManager::ThreadManager(const rclcpp::NodeOptions & options)
	: rclcpp::Node("thread_manager", options)
{
	RCLCPP_INFO(get_logger(), "thread_manger constructor.");
}

ThreadManager::~ThreadManager()
{
}

void ThreadManager::start_thread(const std::string & tname)
{
	std::thread thrd = std::thread(&ThreadManager::sleep_for, this, tname, 3600);
	tm_[tname] = thrd.native_handle();
	thrd.detach();
	std::cout << "Thread " << tname << " created: " << std::endl;
}

void ThradManager::stop_thread(const std::string & tname)
{
	ThreadMap::const_iterator it = tm_.find(tname);
	if (it != tm_.end())
	{
		pthread_cancel(it->second);
		tm_.erase(tname);
		std::cout << "Thread " << tname << " killed:" << std::endl;
	}
}
