// 平移相机识别到人的坐标，变换人与相机的距离
#pragma once

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "astra_camera_msgs/msg/coord_person_list.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"
#include <mutex>
#include <queue>

namespace astra_camera{

struct valid_data_range
{
        float min_x;
        float max_x;
        float min_y;
        float max_y;
        float min_z;
        float max_z;
};

class PointsPersonTF: public rclcpp::Node
{
public:
        PointsPersonTF(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~PointsPersonTF();

        void init_params();
        void coord_tf(geometry_msgs::msg::Point&);
        void points_tf(sensor_msgs::msg::PointCloud2&);
        void rotate_x(float &y, float &z, float theta);
        void process(int x1, int y1, int x2, int y2, float theta, int r);

        void cb_coord(const astra_camera_msgs::msg::CoordPersonList::SharedPtr msg);
        void cb_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        bool filter(cv::Mat & mat);
private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
        rclcpp::Subscription<astra_camera_msgs::msg::CoordPersonList>::SharedPtr sub_persons_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
        geometry_msgs::msg::Point point_;
        rclcpp::Logger logger_;
        std::string topic_coodinate_;
        std::string topic_points_;
        std::string topic_pub_;
        float tf_left_;
        float tf_right_;
        float tf_back_;
        std::mutex mtx_;
        std::queue<astra_camera_msgs::msg::CoordPersonList> queue_persons_;
        int queue_size_;
        float z_min_;
        float theta_x_;
        int r_;
        int width_, height_;
        cv::Mat frame;
        std::vector<std::vector<bool>> processed_;
        int data_count_;
        unsigned char *frame_data_;
        valid_data_range data_range_;
        bool filter_bool_ {false};
        int frame_index_points = 0;
        int frame_index_coord = 0;
        int count_has_person = 0;
        int count_no_person = 0;

        // just for test
        bool test_mode_{false};
        int test_x1_, test_y1_, test_x2_, test_y2_;
};
} // end of namespace astra_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(astra_camera::PointsPersonTF)
