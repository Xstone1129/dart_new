#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
// STD 头文件
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip> // 必须包含此头文件
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "light_detector/detector_node.hpp"
namespace rm_auto_light
{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions &options) : Node("detect_node", options)
    {
        // time
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm *now_tm = std::localtime(&now_c);
        std::ostringstream oss;
        oss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
        std::string time_str = oss.str();

        RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
        detector_ = initDetector();
        target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>("/detector/target", rclcpp::SensorDataQoS());

        // Debug Publishers
        debug_ = this->declare_parameter("debug", true);

        if (debug_)
        {
            createDebugPublishers();
        }


        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info",
                                                                                rclcpp::SensorDataQoS(),
                                                                                [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
                                                                                {
                                                                                    cam_center_ = cv::Point2f(camera_info->k[2], camera_info->k[5]);
                                                                                    cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
                                                                                    cam_info_sub_.reset();
                                                                                });
        img_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&DetectorNode::targetWorkCallback, this, std::placeholders::_1));
    }

    std::unique_ptr<Detector> DetectorNode::initDetector()
    {
        InitKalmanFilter();
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        if (param_desc.integer_range.size() < 1)
        {
            param_desc.integer_range.resize(1);
        }
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;
        int binary_thres = declare_parameter("binary_thres", 160, param_desc);


        compensation = static_cast<int>(declare_parameter("compensation", 0));
        auto detector = std::make_unique<Detector>(binary_thres);

        return detector;
    }

    void DetectorNode::targetWorkCallback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        // update params
        compensation = get_parameter("compensation").as_int();
        // 标架上面的相机是装反的
        // 0 表示上下翻转，即垂直翻转
        // 1 表示左右翻转，即水平翻转
        // -1 表示同时上下和左右翻转

        auto light = detectLight(img);
        if (debug_)
        {
            publishDebugImages();
        }

        // populate target message
        auto_aim_interfaces::msg::Target target_msg;
        target_msg.is_detected = light.is_detected ? 1 : 0;
        target_msg.yaw_error = light.is_detected ? (cam_center_.y - static_cast<int>(light.center_point.y) + compensation) : 0;
        target_pub_->publish(target_msg);


        // 画图用于debug
        if (debug_)
        {
            std::string yaw_text = "yaw: " + std::to_string(target_msg.yaw_error);
            cv::putText(detector_->debug_image_, yaw_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 4, cv::Scalar(0, 0, 255), 4);
            cv::line(detector_->debug_image_,
                     cv::Point(0, cam_center_.y + compensation), // 起点
                     cv::Point(cam_center_.x, cam_center_.y + compensation), // 终点
                     cv::Scalar(0, 0, 255), // 线条颜色 (红色)
                     2); // 线条粗细
        }
    }


    // TODO：加入卡尔曼滤波
    void DetectorNode::InitKalmanFilter()
    {
        KF = cv::KalmanFilter(1, 1, 0);

        // Transition matrix (A)
        KF.transitionMatrix = (cv::Mat_<float>(3, 3) << 1);

        // Measurement matrix (H)
        setIdentity(KF.measurementMatrix);

        // Noise covariance matrices (Q and R)
        setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3)); // Process noise (Q)
        setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-1)); // Measurement noise (R)

        // Set initial state and covariance
        setIdentity(KF.errorCovPost, cv::Scalar::all(1000)); // Error covariance
        measurement_ = cv::Mat::zeros(1, 1, CV_32F); // Initialize measurement
        KF.statePost = (cv::Mat_<float>(1, 1) << 0); // Initial state
    }

    void DetectorNode::KalmanUpdate()
    {
        prediction_ = KF.predict();

        KF.correct(measurement_);
    }


    Light DetectorNode::detectLight(const sensor_msgs::msg::Image::SharedPtr img)
    {
        cv::Mat imgROI = cv_bridge::toCvShare(img, "bgr8")->image;
        cv::flip(img_flip_, imgROI, 0);
        return detector_->detect(img_flip_);
    }

    void DetectorNode::createDebugPublishers()
    {
        binary_img_pub_ = image_transport::create_publisher(this, "/detector/binary_img");
        dilate_img_pub_ = image_transport::create_publisher(this, "/detector/dilate_img");
        result_img_pub_ = image_transport::create_publisher(this, "/detector/result_img");
    }


    void DetectorNode::publishDebugImages()
    {
        binary_img_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", detector_->binary_img_).toImageMsg());
        dilate_img_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", detector_->dilate_img_).toImageMsg());
        result_img_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", detector_->debug_image_).toImageMsg());
    }

    void DetectorNode::destroyDebugPublishers()
    {
        binary_img_pub_.shutdown();
        dilate_img_pub_.shutdown();
        result_img_pub_.shutdown();
    }


} // namespace rm_auto_light


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_light::DetectorNode)
