#include "astra_camera/test/points_person_transform.hpp"

using namespace std::placeholders;

namespace astra_camera
{
PointsPersonTF::PointsPersonTF(const rclcpp::NodeOptions & options)
	: rclcpp::Node("points_person_tf", options), logger_(this->get_logger())
{
	RCLCPP_INFO(logger_, "PointsPersonTF constructor.");
	init_params();
	cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto sub_options = rclcpp::SubscriptionOptions();
	sub_options.callback_group = cb_group_;
	sub_persons_ = create_subscription<astra_camera_msgs::msg::CoordPersonList>(
		topic_coodinate_,rclcpp::SensorDataQoS(),
		std::bind(&PointsPersonTF::cb_coord, this, _1), sub_options);
	sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
		topic_points_,rclcpp::QoS(5).best_effort(),
		std::bind(&PointsPersonTF::cb_points, this, _1));
	auto pub_options = rclcpp::PublisherOptions();
	pub_options.callback_group = cb_group_;
	pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_pub_, rclcpp::QoS(5).best_effort(), pub_options);
}

PointsPersonTF::~PointsPersonTF()
{
}

void PointsPersonTF::init_params()
{
	declare_parameter<std::string>("topic_coodinate", "/coord_persons");
	declare_parameter<std::string>("topic_points", "/camera/depth/points");
	declare_parameter<std::string>("topic_pub", "points_cloud_person");
	declare_parameter<float>("tf_left", 0.1);
	declare_parameter<float>("tf_right", 0.1);
	declare_parameter<float>("tf_back", 0.3);
	declare_parameter<int>("queue_size", 5);
	declare_parameter<float>("z_min", 0.9);
	declare_parameter<float>("theta_x", -0.785398);
	declare_parameter<int>("r_", 0);

	topic_coodinate_ = get_parameter("topic_coodinate").get_value<std::string>();
	topic_points_ = get_parameter("topic_points").get_value<std::string>();
	topic_pub_ = get_parameter("topic_pub").get_value<std::string>();
	tf_left_ = get_parameter("tf_left").get_value<float>();
	tf_right_ = get_parameter("tf_right").get_value<float>();
	tf_back_ = get_parameter("tf_back").get_value<float>();
	queue_size_ = get_parameter("queue_size").get_value<int>();
	z_min_ = get_parameter("z_min").get_value<float>();
	theta_x_ = get_parameter("theta_x").get_value<float>();
	r_ = get_parameter("r_").get_value<int>();
	RCLCPP_INFO_STREAM(logger_, "tf_left: " << tf_left_ << ", tf_right_: " << tf_right_
	                                        << ", tf_back_: " << tf_back_
	                                        << ", queue_size: " << queue_size_
	                                        << ", z_min: " << z_min_
	                                        << ", theta_x: " << theta_x_
	                                        << ", r: " << r_);


	// just for test(comment after test)
	declare_parameter<bool>("test_mode", false);
	declare_parameter<int>("test_x1", 100);
	declare_parameter<int>("test_y1", 150);
	declare_parameter<int>("test_x2", 400);
	declare_parameter<int>("test_y2", 300);
	test_mode_ = get_parameter("test_mode").get_value<bool>();
	test_x1_ = get_parameter("test_x1").get_value<int>();
	test_y1_ = get_parameter("test_y1").get_value<int>();
	test_x2_ = get_parameter("test_x2").get_value<int>();
	test_y2_ = get_parameter("test_y2").get_value<int>();
	RCLCPP_INFO(logger_, "test_mode: %s", test_mode_? "true" : "false");
}

void PointsPersonTF::cb_coord(const astra_camera_msgs::msg::CoordPersonList::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mtx_);
	if (msg->persons.size() == 0)
	{
		return;
	}
	if (queue_persons_.size() < queue_size_)
	{
		queue_persons_.push(*msg);
	}
	else
	{
		queue_persons_.pop();
		queue_persons_.push(*msg);
	}
}

void PointsPersonTF::cb_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(mtx_);
	sensor_msgs::msg::PointCloud2 msg_pub;
	msg_pub = *msg;
	if (test_mode_ || !queue_persons_.empty())
	{
		height_ = msg->height;
		width_ = msg->width;
		data_count_ = msg->row_step * height_;
		frame_data_ = new unsigned char[data_count_];
		for(int count_index = 0; count_index < data_count_; count_index++)
		{
			frame_data_[count_index] = msg->data[count_index];
		}
		frame = cv::Mat(height_, width_, CV_32FC4, frame_data_);
		processed_.clear();
		processed_ = std::vector<std::vector<bool> >(height_, std::vector<bool>(width_, false));
	}
	if (test_mode_)
	{
		process(test_x1_, test_y1_, test_x2_, test_y2_, theta_x_, r_);
	}
	else if(!test_mode_ && !queue_persons_.empty())
	{
		auto person_list = queue_persons_.front();
		queue_persons_.pop();
		int size = person_list.persons.size();
		int x1,y1, x2,y2;
		for (int i = 0; i < size; i++)
		{
			x1 = person_list.persons[i].x1;
			y1 = person_list.persons[i].y1;
			x2 = person_list.persons[i].x2;
			y2 = person_list.persons[i].y2;
			process(x1, y1, x2, y2, theta_x_, r_);
		}
	}
	if (test_mode_ || !queue_persons_.empty())
	{
		msg_pub.data.clear();
		for (int i = 0; i < data_count_; i++)
		{
			msg_pub.data.push_back(frame.data[i]);
		}
		delete[] frame_data_;
		frame_data_ = NULL;
	}
	pub_points_->publish(msg_pub);
}

void PointsPersonTF::process(int x1, int y1, int x2, int y2, float theta, int r)
{
	int start_y, end_y;
	if (r_ == 0)
	{
		start_y = y1;
		end_y = y2;
	}
	else
	{
		if (r < 0)
		{
			r = 10;
		}
		start_y = std::max((y1 + y2) / 2  - r_, y1);
		end_y = std::min((y1 + y2) / 2 + r_, y2);
	}
	for (int j = start_y; j <= end_y; j++)
	{
		for (int k = x1; k <= x2; k++)
		{
			cv::Vec4f point4f = frame.at<cv::Vec4f>(j,k);
			if (processed_[j][k])
			{
				continue;
			}
			else if(point4f[0] == point4f[0] && point4f[1] == point4f[1] && point4f[2] == point4f[2])
			{
				float x, y, z;                 // x=>right , y=>down, z=> front;
				x = point4f[0];
				y = point4f[1];
				z = point4f[2];
				rotate_x(y, z, theta_x_);
				z = std::max(z_min_, z - tf_back_);
				if (k < (x1 + x2) / 2)
				{
					x -= tf_left_;
				}
				if(k > (x1 + x2) / 2)
				{
					x += tf_right_;
				}
				rotate_x(y, z, -theta_x_);
				point4f = {x, y, z, 0.};
				frame.at<cv::Vec4f>(j, k) = point4f;
			}
			processed_[j][k] = true;
		}
	}
}

void PointsPersonTF::rotate_x(float &y, float &z, float theta)
{
	// cos(a+b) = cos(a)*cos(b) - sin(a)*sin(b)
	// sin(a+b) = sin(a)*cos(b) + cos(a)*sin(b)
	float y_, z_;
	y_ = y * cos(theta) - z * sin(theta);
	z_ = z * cos(theta) + y * sin(theta);
	y = y_;
	z = z_;
}

} // end of namespace astra_camera