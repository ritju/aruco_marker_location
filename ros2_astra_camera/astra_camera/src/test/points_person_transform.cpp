#include "astra_camera/test/points_person_transform.hpp"

using namespace std::placeholders;

namespace astra_camera
{
PointsPersonTF::PointsPersonTF(const rclcpp::NodeOptions & options)
	: rclcpp::Node("points_person_tf", options), logger_(this->get_logger())
{
	RCLCPP_INFO(logger_, "PointsPersonTF constructor.");
	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	init_params();
	cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	auto sub_options = rclcpp::SubscriptionOptions();
	sub_options.callback_group = cb_group_;
	sub_persons_ = create_subscription<astra_camera_msgs::msg::CoordPersonList>(
		topic_coodinate_,rclcpp::QoS(10).reliable(),
		std::bind(&PointsPersonTF::cb_coord, this, _1), sub_options);
	sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
		topic_points_,rclcpp::QoS(5).best_effort(),
		std::bind(&PointsPersonTF::cb_points, this, _1), sub_options);
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
	declare_parameter<std::string>("topic_points", "/camera1/depth/points");
	declare_parameter<std::string>("topic_pub", "points_cloud_person");
	declare_parameter<float>("tf_left", 0.1);
	declare_parameter<float>("tf_right", 0.1);
	declare_parameter<float>("tf_back", 0.3);
	declare_parameter<int>("queue_size", 5);
	declare_parameter<float>("z_min", 0.9);
	declare_parameter<float>("theta_x", -0.785398);
	declare_parameter<int>("r", 0);
	declare_parameter<float>("valid_x_min", -2.0);
	declare_parameter<float>("valid_x_max", -2.0);
	declare_parameter<float>("valid_y_min", -0.2);
	declare_parameter<float>("valid_y_max", 1.5);
	declare_parameter<float>("valid_z_min", 0.3);
	declare_parameter<float>("valid_z_max", 4.0);
	declare_parameter<bool>("filter_bool", false);
	declare_parameter<float>("life_time", 1.0);
	declare_parameter<std::string>("topic_frame", "camera1_color_frame");

	topic_coodinate_ = get_parameter("topic_coodinate").get_value<std::string>();
	topic_points_ = get_parameter("topic_points").get_value<std::string>();
	topic_frame_ = get_parameter("topic_frame").get_value<std::string>();
	topic_pub_ = get_parameter("topic_pub").get_value<std::string>();
	tf_left_ = get_parameter("tf_left").get_value<float>();
	tf_right_ = get_parameter("tf_right").get_value<float>();
	tf_back_ = get_parameter("tf_back").get_value<float>();
	queue_size_ = get_parameter("queue_size").get_value<int>();
	z_min_ = get_parameter("z_min").get_value<float>();

	geometry_msgs::msg::TransformStamped transform_stamped;
	if (getTransform(std::string("base_link"), topic_frame_, transform_stamped))
	{
		tf2::Stamped<tf2::Transform> cameraToReference;
		cameraToReference.setIdentity();
		tf2::fromMsg(transform_stamped, cameraToReference);
		auto tf_tmp = static_cast<tf2::Transform>(cameraToReference);
		
		tf2::Matrix3x3 m(tf_tmp.getRotation());
    	double roll, pitch, yaw;
    	m.getRPY(roll, pitch, yaw);
		theta_x_ = pitch;

		RCLCPP_INFO(get_logger(), "theta_x from tf: %f", theta_x_);
		theta_x_ = -theta_x_;
		RCLCPP_INFO(get_logger(), "-theta_x from tf: %f", theta_x_);
		theta_x_get_from_tf = true;
	} 
	else
	{
		theta_x_ = get_parameter("theta_x").get_value<float>();
		RCLCPP_INFO(get_logger(), "theta_x from param file: %f", theta_x_);
		theta_x_ = false;
	}
	

	r_ = get_parameter("r").get_value<int>();
	data_range_.min_x = get_parameter("valid_x_min").get_value<float>();
	data_range_.max_x = get_parameter("valid_x_max").get_value<float>();
	data_range_.min_y = get_parameter("valid_y_min").get_value<float>();
	data_range_.max_y = get_parameter("valid_y_max").get_value<float>();
	data_range_.min_z = get_parameter("valid_z_min").get_value<float>();
	data_range_.max_z = get_parameter("valid_z_max").get_value<float>();
	filter_bool_ = get_parameter("filter_bool").get_value<bool>();
	life_time_ = get_parameter("life_time").get_value<float>();
	RCLCPP_INFO_STREAM(logger_, "tf_left: " << tf_left_ << ", tf_right_: " << tf_right_
	                                        << ", tf_back_: " << tf_back_
	                                        << ", queue_size: " << queue_size_
	                                        << ", z_min: " << z_min_
	                                        << ", theta_x: " << theta_x_
	                                        << ", r: " << r_
	                                        << ", valid_x_min: " << data_range_.min_x
	                                        << ", valid_x_max: " << data_range_.max_x
	                                        << ", valid_y_min: " << data_range_.min_y
	                                        << ", valid_y_max: " << data_range_.max_y
	                                        << ", valid_z_min: " << data_range_.min_z
	                                        << ", valid_z_max: " << data_range_.max_z
	                                        << ", filter_bool: " << filter_bool_
	                                        << ", life_time: " << life_time_
											<< ", topic_frame: " << topic_frame_.c_str()
	                   );


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
	// frame_index_coord++;
	std::lock_guard<std::mutex> lock(mtx_);
	if (msg->persons.size() == 0)
	{
		count_no_person++;
		return;
	}
	count_has_person++;
	if (queue_persons_.size() == queue_size_)
	{
		queue_persons_.pop();
	}
	queue_persons_.push(*msg);
}

void PointsPersonTF::cb_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	if(!theta_x_get_from_tf)
	{
		geometry_msgs::msg::TransformStamped transform_stamped;
		if (getTransform(std::string("base_link"), topic_frame_, transform_stamped))
		{
			tf2::Stamped<tf2::Transform> cameraToReference;
			cameraToReference.setIdentity();
			tf2::fromMsg(transform_stamped, cameraToReference);
			auto tf_tmp = static_cast<tf2::Transform>(cameraToReference);
			
			tf2::Matrix3x3 m(tf_tmp.getRotation());
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
			theta_x_ = pitch;

			RCLCPP_INFO(get_logger(), "theta_x from tf: %f", theta_x_);
			theta_x_ = -theta_x_;
			RCLCPP_INFO(get_logger(), "-theta_x from tf: %f", theta_x_);
			theta_x_get_from_tf = true;
		} 
		else
		{
			theta_x_ = get_parameter("theta_x").get_value<float>();
			RCLCPP_INFO(get_logger(), "theta_x from param file: %f", theta_x_);
			theta_x_ = false;
		}
	}
	
	// frame_index_points++;
	// RCLCPP_INFO_STREAM(logger_,
	//                    "\n"
	//                    <<"frame_index_coord:     " << frame_index_coord
	//                    << "\nframe_index_points: " << frame_index_points
	//                    << "\ncount_has_person:   " << count_has_person
	//                    << "\ncount_no_person:    " << count_no_person);
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
		processed_ = std::vector<std::vector<bool> >(height_, std::vector<bool>(width_, false));

		if (test_mode_)
		{
			process(test_x1_, test_y1_, test_x2_, test_y2_, theta_x_, r_);
		}
		else if(!queue_persons_.empty())
		{
			// RCLCPP_INFO(logger_, "************ entry ************");
			astra_camera_msgs::msg::CoordPersonList person_list;
			int size;
			std::lock_guard<std::mutex> lock(mtx_);
			{
				person_list = queue_persons_.front();
				queue_persons_.pop();
			}
			size = person_list.persons.size();
			int x1,y1, x2,y2;
			for (int i = 0; i < size; i++)
			{
				x1 = person_list.persons[i].x1;
				y1 = person_list.persons[i].y1;
				x2 = person_list.persons[i].x2;
				y2 = person_list.persons[i].y2;
				// RCLCPP_INFO_STREAM(logger_, "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);
				process(x1, y1, x2, y2, theta_x_, r_);
			}
			last_coord_ = person_list;
			last_time_ = this->get_clock()->now().seconds();
			RCLCPP_DEBUG(logger_, "---------------------------------");
			RCLCPP_DEBUG(logger_, "last_time: %f", last_time_);
		}
		msg_pub.data.clear();
		if(filter_bool_)
		{
			filter(frame);
		}
		for (int i = 0; i < data_count_; i++)
		{
			msg_pub.data.push_back(frame.data[i]);
		}
		delete[] frame_data_;
		frame_data_ = NULL;
	}
	else
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
		processed_ = std::vector<std::vector<bool> >(height_, std::vector<bool>(width_, false));

		double now_time = this->get_clock()->now().seconds();
		double delta_time = now_time - last_time_;
		RCLCPP_DEBUG(logger_, "*********************");
		RCLCPP_DEBUG(logger_, "last_time: %f", last_time_);
		RCLCPP_DEBUG(logger_, "now_time: %f", now_time);
		RCLCPP_DEBUG(logger_, "delta time: %f", delta_time);
		if (delta_time < life_time_)
		{
			// RCLCPP_INFO(logger_, "delta time: %f", delta_time);

			astra_camera_msgs::msg::CoordPersonList person_list;
			int size;
			person_list = last_coord_;
			size = person_list.persons.size();
			int x1,y1, x2,y2;
			for (int i = 0; i < size; i++)
			{
				x1 = person_list.persons[i].x1;
				y1 = person_list.persons[i].y1;
				x2 = person_list.persons[i].x2;
				y2 = person_list.persons[i].y2;
				// RCLCPP_INFO_STREAM(logger_, "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);
				process(x1, y1, x2, y2, theta_x_, r_);
			}

			msg_pub.data.clear();
			for (int i = 0; i < data_count_; i++)
			{
				msg_pub.data.push_back(frame.data[i]);
			}
		}
		delete[] frame_data_;
		frame_data_ = NULL;
	}
	processed_.clear();
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

	// fix bug for coord ranges
	start_y = std::min(std::max(start_y, 0), height_ - 1);
	end_y = std::min(std::max(end_y, 0), height_ - 1);
	x1 = std::min(std::max(x1, 0), width_ - 1);
	x2 = std::min(std::max(x2, 0), width_ - 1);
	
	// RCLCPP_INFO(logger_, "start_y: %d, end_y: %d", start_y, end_y );
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
				float x, y, z;
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
	// theta: the angle from z axis move to y axis
	float y_, z_;
	y_ = y * cos(theta) - z * sin(theta);
	z_ = z * cos(theta) + y * sin(theta);
	y = y_;
	z = z_;
}

bool PointsPersonTF::filter(cv::Mat &mat)
{
	bool ret = true;
	if (mat.type() == CV_32FC4 && mat.rows == height_ && mat.cols == width_)
	{
		for (int row = 0; row < height_; row++)
		{
			for (int col = 0; col < width_; width_++)
			{
				auto point4f = mat.at<cv::Vec4f>(row, col);
				if(point4f[0] == point4f[0] && point4f[1] == point4f[1] && point4f[2] == point4f[2])
				{
					if (point4f[0] > data_range_.min_x && point4f[0] < data_range_.max_x
					    && point4f[1] > data_range_.min_y && point4f[1] < data_range_.max_y
					    && point4f[2] > data_range_.min_z && point4f[2] < data_range_.max_z)
					{
						continue;
					}
					else
					{
						point4f[0]=point4f[1]=point4f[2]=-10;
						mat.at<cv::Vec4f>(row, col) = point4f;
					}
				}
			}
		}
	}
	else
	{
		ret = false;
	}
	return ret;
}

bool PointsPersonTF::getTransform(
	const std::string & refFrame, const std::string & childFrame,
	geometry_msgs::msg::TransformStamped & transform)
{
	std::string errMsg;

	if (!tf_buffer_->canTransform(
		    refFrame, childFrame, tf2::TimePointZero,
		    tf2::durationFromSec(0.5), &errMsg))
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
		return false;
	} else {
		try {
			transform = tf_buffer_->lookupTransform(
				refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
					0.5));
		} catch (const tf2::TransformException & e) {
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
			return false;
		}
	}
	return true;
}

} // end of namespace astra_camera