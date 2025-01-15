#pragma once

// ROS相关头文件
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// OpenCV相关头文件
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// Eigen库（矩阵运算）
#include <Eigen/Dense>

// ROS与OpenCV的桥接库
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// TF2库（用于坐标变换）
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 标准库头文件
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 项目特定的头文件
#include <light_detector/detector.hpp>
#include <light_detector/light.hpp>
#include "auto_aim_interfaces/msg/target.hpp"

// 命名空间
namespace rm_auto_light
{
    // DetectorNode类定义
    class DetectorNode : public rclcpp::Node
    {
    public:
        explicit DetectorNode(const rclcpp::NodeOptions &options);
        ~DetectorNode()
        {}

        std::unique_ptr<Detector> initDetector();

        void targetWorkCallback(const sensor_msgs::msg::Image::SharedPtr msg);

        Light detectLight(const sensor_msgs::msg::Image::SharedPtr img);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_msg_sub_; // 图像消息订阅

        // 相机相关
        cv::Point2f cam_center_; // 相机中心
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_; // 相机信息
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_; // 相机信息订阅
        cv::Mat img_flip_;

        std::unique_ptr<Detector> detector_;

        // debug
        bool debug_;
        rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_; // 目标发布
        image_transport::Publisher binary_img_pub_; // 二值图发布
        image_transport::Publisher dilate_img_pub_; // 膨胀图发布
        image_transport::Publisher result_img_pub_; // 结果图发布
        void publishDebugImages();
        void createDebugPublishers();
        void destroyDebugPublishers();

        // 卡尔曼滤波器
        cv::KalmanFilter KF; // 卡尔曼滤波器
        cv::Mat measurement_; // 测量值
        cv::Mat prediction_; // 预测值
        void InitKalmanFilter();
        void KalmanUpdate();


        enum class MODE
        {
            NOTHING = 0,
            AUTO_AIM,
        };
        enum class NUMBER
        {
            ONE=1,
            TWO,
            THREE,
            FOUR,
        };
        
        int mode_;
        int number_;
        float compensation_;
    };
} // namespace rm_auto_light
