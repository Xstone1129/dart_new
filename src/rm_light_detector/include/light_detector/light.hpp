#pragma once
// #include <cv_bridge/cv_bridge.h>
// #include <rmw/qos_profiles.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/convert.h>
//
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <image_transport/image_transport.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/imgproc.hpp>
// #include <rclcpp/duration.hpp>
// #include <rclcpp/qos.hpp>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include "opencv2/video/tracking.hpp"

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h> // 这是 tf2::Quaternion 的定义
#include <vector>


namespace rm_auto_light
{
    struct Light
    {
        Light() : is_detected(false), g_confidence(0.0f)
        {}
        Light(const cv::Point2f &point, bool detected, float confidence) : center_point(point), is_detected(detected), g_confidence(confidence)
        {}

        bool is_detected=false; // 是否检测到光源
        float g_confidence; // 绿色置信度
        cv::Rect box; // 光源的边界框
        cv::Point2f center_point; // 光源的中心点
    };
} // namespace rm_auto_light
