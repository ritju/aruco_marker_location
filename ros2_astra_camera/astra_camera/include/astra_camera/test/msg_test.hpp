// test msg reliability of camera msg
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace astra_camera{
class MsgTest : public rclcpp::Node
{
public:
explicit MsgTest(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
~MsgTest();

void init();
void display(const sensor_msgs::msg::Image::SharedPtr & msg, int index);
void cb_check_frame();

private:
std::vector<std::string> topics_image_sub_;
std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
int count_topics;
int count;
std::shared_ptr<rclcpp::CallbackGroup> cb_group_ptr;
std::vector<cv::Mat> image_vec;
std::vector<float> fps_vec_;
std::vector<std::pair<float, float>> frame_time_last_and_now;
std::vector<double> last_received_frame_time;
rclcpp::TimerBase::SharedPtr check_frame_received_timer;
double timeout_receive;
unsigned char * data;
};
} // namespace astra_camera




