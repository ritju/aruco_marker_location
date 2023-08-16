#include "astra_camera/test/thread_manager.hpp"

namespace astra_camera {

ThreadManager::ThreadManager(const rclcpp::NodeOptions & options)
	: rclcpp::Node("thread_manager", options)
{
	RCLCPP_INFO(get_logger(), "thread_manger constructor.");
	init();
	loop();
	RCLCPP_INFO(get_logger(), "success %d / %d, success_rate: %.2f%%",success_count, loop_number, (float)success_count / loop_number * 100 );
}

ThreadManager::~ThreadManager()
{
}

void ThreadManager::start_thread(const std::string & tname)
{
	RCLCPP_INFO(get_logger(), "Thread %s created.", tname.c_str());
	devices_connected_number = 0;
	success = false;
	std::thread thrd = std::thread(&ThreadManager::work_for, this, tname, 3);
	tm_[tname] = thrd.native_handle();
	thrd.join();
}

void ThreadManager::stop_thread(const std::string & tname)
{
	RCLCPP_INFO(get_logger(), "stopping ...");
	ThreadMap::const_iterator it = tm_.find(tname);
	if (it != tm_.end())
	{
		pthread_cancel(it->second);
		tm_.erase(tname);
		RCLCPP_INFO(get_logger(), "Thread %s killed.", tname.c_str());
		std::cout << std::endl;
	}
}

void ThreadManager::work_for(const std::string & tname, int num)
{
	prctl(PR_SET_NAME, tname.c_str(), 0, 0, 0);
	openni::Status rc;
	int i;
	for(i = 1; i < 101; i++)
	{
		rc = openni::OpenNI::initialize();
		if(rc == openni::Status::STATUS_OK)
			break;
	}
	RCLCPP_INFO(get_logger(), "Try initialize %d times", i);
	auto connected_cb = [&](const openni::DeviceInfo* device_info) {
				    auto device = std::make_shared<openni::Device>();
				    auto uri = device_info->getUri();
				    std::cout << "URI: " << uri << std::endl;
				    int i = 1;
				    openni::Status rc;
				    for(; i < 101; i++)
				    {
					    rc = device->open(uri);
					    if(rc == openni::Status::STATUS_OK)
						    break;
				    }
				    std::cout << "connect " << i << " times." << std::endl;
				    if (rc == openni::STATUS_OK)
				    {
					    devices_connected_number++;
					    device->close();
					    if(devices_connected_number == 3)
					    {
						    RCLCPP_INFO(get_logger(), "======== success =======");
						    success_count++;
					    }
				    }
			    };
	auto disconnected_cb = [&](const openni::DeviceInfo* device_info) {
				       std::cout << "device " << device_info->getUri() << " disconnected" << std::endl;
			       };
	auto device_listener_ =
		std::make_unique<astra_camera::DeviceListener>(connected_cb, disconnected_cb);
	RCLCPP_INFO(get_logger(), "before shutdown()");
	// openni::OpenNI::shutdown();
	RCLCPP_INFO(get_logger(), "before shutdown()");
}

void ThreadManager::init()
{
	loop_number = declare_parameter<int>("loop_number", 5);
}

void ThreadManager::loop()
{
	RCLCPP_INFO(get_logger(), "loop_number: %d", loop_number);
	for (int cur_num = 1; cur_num <= loop_number; cur_num++)
	{
		RCLCPP_INFO(get_logger(), "****************** %d ******************", cur_num);
		std::string tname = "start_astra_camera "+std::to_string(cur_num);
		start_thread(tname);
		stop_thread(tname);
	}
}

} // end of namespace astra_camera
