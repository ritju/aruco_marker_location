#include "astra_camera/test/msg_test.hpp"

namespace astra_camera {

using namespace std::chrono_literals;

MsgTest::MsgTest(const rclcpp::NodeOptions& options)
	: rclcpp::Node("msg_test", options)
{
	RCLCPP_INFO(get_logger(), "MsgTest constructor.");
	init();
}

MsgTest::~MsgTest()
{
}

void MsgTest::init()
{
	topics_image_sub_ = this->declare_parameter<std::vector<std::string> >("topics_sub", std::vector<std::string>());
	timeout_receive = this->declare_parameter<double>("timeout_receive", 3.0);
	count = topics_image_sub_.size();
        
	std::cout << "count: " << count << std::endl;
	for(int i = 0; i < count; i++)
	{
		std::cout << "topic: " << topics_image_sub_[i] << std::endl;
		cv::namedWindow(topics_image_sub_[i], cv::WindowFlags::WINDOW_NORMAL);
	}
        image_vec.resize(count);
	last_received_frame_time.resize(count, this->now().seconds());
	fps_vec_.resize(count, 10);
	check_frame_received_timer = this->create_wall_timer(5s, [&](){cb_check_frame();});

	cb_group_ptr = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto options = rclcpp::SubscriptionOptions();
	options.callback_group = cb_group_ptr;
	for(int i = 0; i < count; i++)
	{
                std::string topic = topics_image_sub_[i];
		auto sub = create_subscription<sensor_msgs::msg::Image>(topic, rclcpp::SensorDataQoS(),
		                                                        [&, i](const sensor_msgs::msg::Image::SharedPtr msg){
				display(msg, i);
			}, options);
		image_subs_.push_back(sub);
	}

}

void MsgTest::display(const sensor_msgs::msg::Image::SharedPtr &msg, int index)
{
        last_received_frame_time[index] = now().seconds();
	std::string encoding = msg->encoding;
	int height = msg->height;
	int width = msg->width;
	int row_step = msg->step;
	int data_count = row_step * height;
	data = new unsigned char[data_count];
	for(int data_index = 0; data_index < data_count; data_index++)
	{
		data[data_index] = msg->data[data_index];
	}
	int mat_type = CV_8UC3;
	if(encoding.compare(std::string("rgb8")) == 0)
	{
		mat_type = (int)CV_8UC3;
	}
	else if(encoding.compare(std::string("16UC1")) == 0)
	{
		mat_type = (int)CV_16UC1;
	}
	image_vec[index] = cv::Mat(height, width, mat_type, data);
	cv::imshow(topics_image_sub_[index], image_vec[index]);
	cv::waitKey(10);
	delete data;
}

void MsgTest::cb_check_frame()
{
	for (int index = 0; index < count; index++)
	{
		double now_time = now().seconds();
		double delay = now_time - last_received_frame_time[index];
		if(delay > timeout_receive)
		{
			RCLCPP_WARN(get_logger(), "topic %s timeout!", topics_image_sub_[index].c_str());
		}
		fps_vec_[index] = 1.0 / delay;
		RCLCPP_INFO(get_logger(), "topic %s => fps: %.1f", topics_image_sub_[index].c_str(), fps_vec_[index]);
	}
}
} // namespace astra_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::MsgTest)