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
#include <opencv2/core/types.hpp>
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
    const int RED = 0;
    const int BLUE = 1;
    const int GREEN = 2;

    enum class ArmorType
    {
        SMALL,
        LARGE,
        INVALID
    };
    struct GreenLight : public cv::RotatedRect
    {
        GreenLight() = default;
        GreenLight(cv::RotatedRect box) : cv::RotatedRect(box)
        {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        bool is_detected = false; // 是否检测到光源
        float g_confidence; // 绿色置信度
        cv::Point2f center_point, top, bottom; // 光源的中心点
        double length, width;
        float tilt_angle;
    };

    struct Light : public cv::RotatedRect
    {
        Light() = default;
        explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
        {
            cv::Point2f p[4];
            box.points(p);
            std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
            top = (p[0] + p[1]) / 2;
            bottom = (p[2] + p[3]) / 2;

            length = cv::norm(top - bottom);
            width = cv::norm(p[0] - p[1]);

            tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
            tilt_angle = tilt_angle / CV_PI * 180;
        }

        int color;
        cv::Point2f top, bottom;
        double length;
        double width;
        float tilt_angle;
    };
    struct Armor
    {
        Armor() = default;
        Armor(const Light &l1, const Light &l2)
        {
            if (l1.center.x < l2.center.x)
            {
                left_light = l1, right_light = l2;
            }
            else
            {
                left_light = l2, right_light = l1;
            }
            center = (left_light.center + right_light.center) / 2;
        }

        // Light pairs part
        Light left_light, right_light;
        cv::Point2f center;
        ArmorType type;

        // Number part
        cv::Mat number_img;
        std::string number;
        float confidence;
        std::string classfication_result;
    };
} // namespace rm_auto_light
