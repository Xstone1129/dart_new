// 标准库头文件
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/types.hpp>
#include <stdio.h>
#include <string>
#include <vector>

// ROS 相关头文件
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// OpenCV 相关头文件
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// ROS 与 OpenCV 之间的桥接
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// 项目特定的头文件
#include "light_detector/detector.hpp"
#include "light_detector/light.hpp"

namespace rm_auto_light
{

    Detector::Detector(const int &bin_thres, const int &color, const LightParams &l, const ArmorParams &a, GreenLightParams &g) :
        binary_thres(bin_thres), detect_color(color), l_(l), a_(a), g_(g)
    {
        RCLCPP_INFO(rclcpp::get_logger("rm_auto_light"), "初始化好了，二值化阈值：%d", binary_thres);
    }

    cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img)
    {
        // 输入有效性检查
        if (rgb_img.empty())
        {
            std::cerr << "ERROR: preprocessImage() received empty image!" << std::endl;
            return cv::Mat();
        }

        cv::Mat gray_img, binary_image;

        try
        {
            // 颜色空间转换
            cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

            // 二值化
            cv::threshold(gray_img, binary_image, binary_thres, 255, cv::THRESH_BINARY);
        }
        catch (cv::Exception &e)
        {
            std::cerr << "OpenCV Exception: " << e.what() << std::endl;
            return cv::Mat();
        }

        return binary_image;
    }

    // 检测函数
    GreenLight Detector::detect(const cv::Mat &input)
    {
        lights_.clear();
        green_lights_.clear();
        // armors_.clear();armors每一个回合会重赋值 不用清空
        debug_image_ = input;
        binary_img_ = preprocessImage(input);
        armors_ = matchLights(lights_);
        drawResults(debug_image_);
        return findGreenLight(input, binary_img_);
    }

    // 计算给定区域的绿色置信度
    std::pair<int, double> Detector::findColor(const cv::Mat &img, const std::vector<cv::Point> &contour)
    {
        double sum_b = 0, sum_g = 0, sum_r = 0;
        for (const cv::Point &pt : contour)
        {
            cv::Vec3b color = img.at<cv::Vec3b>(pt);
            sum_r += color[2];
            sum_g += color[1];
            sum_b += color[0];
        }
        // 计算平均颜色
        int num_points = contour.size();
        if (num_points == 0)
            return {-1, -1}; // 避免除以零

        double avg_r = sum_r / num_points;
        double avg_g = sum_g / num_points;
        double avg_b = sum_b / num_points;


        return 
        (avg_r >= avg_b && avg_r >= avg_g) ? std::make_pair(RED, avg_r): (avg_b >= avg_g)? std::make_pair(BLUE, avg_b): std::make_pair(GREEN, avg_g);
    }

    GreenLight Detector::findGreenLight(const cv::Mat &img, const cv::Mat &binary_image)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 10.0)
                continue; // 跳过过小的轮廓

            cv::RotatedRect r_rect = cv::minAreaRect(contour);
            cv::Rect minRect = r_rect.boundingRect();
            auto light = Light(r_rect);
            auto green_light = GreenLight(r_rect);
            const cv::Rect img_rect(0, 0, img.cols, img.rows);
            if ((minRect & img_rect) != minRect)
                continue;

            if (isLight(light))
            {
                auto color_confidence = findColor(img, contour);
                if (color_confidence.first != GREEN)
                {
                    light.color = color_confidence.first;
                    lights_.emplace_back(light);
                }
            }
            if (isGreenLight(green_light))
            {
                auto color_confidence = findColor(img, contour);
                if (color_confidence.first == GREEN)
                {
                    GreenLight green_light;
                    cv::Point2f center;
                    cv::Moments moments = cv::moments(contour);
                    center = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
                    green_light.g_confidence = color_confidence.second;
                    green_light.center_point = center;
                    green_light.is_detected = true; // 标记为已检测到光源
                    green_lights_.emplace_back(green_light);
                }
            }
        }

        if (!green_lights_.empty())
        {
            auto best_green_light = std::max_element(green_lights_.begin(), green_lights_.end(),
                                                     [](const GreenLight &a, const GreenLight &b) { return a.g_confidence < b.g_confidence; });
            green_light_ = *best_green_light;
            return *best_green_light; // 返回最有可能的光源
        }
        green_light_ = GreenLight(); // 包含 is_detected = false
        return GreenLight(); // 包含 is_detected = false
    }

    bool Detector::isLight(const Light &light)
    {
        // The ratio of light (short side / long side)
        float ratio = light.width / light.length;
        bool ratio_ok = l_.min_ratio < ratio && ratio < l_.max_ratio;
        bool angle_ok = light.tilt_angle < l_.max_angle;
        bool is_light = ratio_ok && angle_ok;

        // Fill in debug information
        // auto_aim_interfaces::msg::DebugLight light_data;
        // light_data.center_x = light.center.x;
        // light_data.ratio = ratio;
        // light_data.angle = light.tilt_angle;
        // light_data.is_light = is_light;
        // this->debug_lights.data.emplace_back(light_data);

        return is_light;
    }

    bool Detector::isGreenLight(const GreenLight &GreenLight)
    {
        // The ratio of light (short side / long side)
        float ratio = GreenLight.width / GreenLight.length;
        bool ratio_ok = g_.min_ratio < ratio && ratio < g_.max_ratio;
        bool angle_ok = GreenLight.tilt_angle < g_.max_angle;
        bool is_grenn_light = ratio_ok && angle_ok;

        // Fill in debug information
        // auto_aim_interfaces::msg::DebugLight light_data;
        // light_data.center_x = light.center.x;
        // light_data.ratio = ratio;
        // light_data.angle = light.tilt_angle;
        // light_data.is_light = is_light;
        // this->debug_lights.data.emplace_back(light_data);

        return is_grenn_light;
    }
    std::vector<Armor> Detector::matchLights(const std::vector<Light> &lights)
    {
        std::vector<Armor> armors;
        // this->debug_armors.data.clear();

        // Loop all the pairing of lights
        for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++)
        {
            for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++)
            {
                if (light_1->color != detect_color || light_2->color != detect_color)
                    continue;

                if (containLight(*light_1, *light_2, lights))
                {
                    continue;
                }

                auto type = isArmor(*light_1, *light_2);
                if (type != ArmorType::INVALID)
                {
                    auto armor = Armor(*light_1, *light_2);
                    armor.type = type;
                    armors.emplace_back(armor);
                }
            }
        }

        return armors;
    }

    bool Detector::containLight(const Light &light_1, const Light &light_2, const std::vector<Light> &lights)
    {
        auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
        auto bounding_rect = cv::boundingRect(points);

        for (const auto &test_light : lights)
        {
            if (test_light.center == light_1.center || test_light.center == light_2.center)
                continue;

            if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) || bounding_rect.contains(test_light.center))
            {
                return true;
            }
        }

        return false;
    }
    ArmorType Detector::isArmor(const Light &light_1, const Light &light_2)
    {
        // Ratio of the length of 2 lights (short side / long side)
        float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length : light_2.length / light_1.length;
        bool light_ratio_ok = light_length_ratio > a_.min_light_ratio;

        // Distance between the center of 2 lights (unit : light length)
        float avg_light_length = (light_1.length + light_2.length) / 2;
        float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
        bool center_distance_ok = (a_.min_small_center_distance <= center_distance && center_distance < a_.max_small_center_distance) ||
            (a_.min_large_center_distance <= center_distance && center_distance < a_.max_large_center_distance);

        // Angle of light center connection
        cv::Point2f diff = light_1.center - light_2.center;
        float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
        bool angle_ok = angle < a_.max_angle;

        bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

        // Judge armor type
        ArmorType type;
        if (is_armor)
        {
            type = center_distance > a_.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
        }
        else
        {
            type = ArmorType::INVALID;
        }

        // Fill in debug information
        // auto_aim_interfaces::msg::DebugArmor armor_data;
        // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
        // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
        // armor_data.light_ratio = light_length_ratio;
        // armor_data.center_distance = center_distance;
        // armor_data.angle = angle;
        // this->debug_armors.data.emplace_back(armor_data);

        return type;
    }

    void Detector::drawResults(cv::Mat &img)
    {

        // Draw Green Light
        if (green_light_.is_detected)
        {
            cv::circle(debug_image_, green_light_.center_point, 5, cv::Scalar(0, 0, 255), -1); // 红色圆点
            std::string score_text = "Score: " + std::to_string(green_light_.g_confidence);
        }

        // Draw Lights
        for (const auto &light : lights_)
        {
            cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
            cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
            auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
            cv::line(img, light.top, light.bottom, line_color, 1);
        }

        // Draw armors
        for (const auto &armor : armors_)
        {
            cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
            cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
        }

        // Show numbers and confidence
        for (const auto &armor : armors_)
        {
            cv::putText(img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        }
    }

    // TODO:加入白灯检测，但是目前的绿灯跟白灯太接近
    //  bool Detector::isWhiteLight(int sum_r, int sum_g, int sum_b)
    //  {
    //     return sum_r > white_avg_r_ && sum_g > white_avg_g_ && sum_b > white_avg_b_ && std::abs(sum_r - sum_g) < white_color_tolerance_ &&
    //     std::abs(sum_r - sum_b) < white_color_tolerance_ &&
    //         std::abs(sum_g - sum_b) < white_color_tolerance_;
    // }


} // namespace rm_auto_light
