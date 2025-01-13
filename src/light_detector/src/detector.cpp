// 标准库头文件
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
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

namespace rm_auto_light
{

    Detector::Detector(const int &bin_thres) : binary_thres(bin_thres)
    {
        RCLCPP_INFO(rclcpp::get_logger("rm_auto_light"), "初始化好了，二值化阈值：%d", binary_thres);
    }

    cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img)
    {
        cv::Mat gray_img, binary_image;
        cv::cvtColor(rgb_img, gray_img, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img, binary_image, binary_thres, 255, cv::THRESH_BINARY);
        return binary_image;
    }

    // 检测函数
    Light Detector::detect(const cv::Mat &input)
    {
        debug_image_ = input;
        binary_img_ = preprocessImage(input);
        return findGreenLight(input, binary_img_);
    }

    // 计算给定区域的绿色置信度
    double Detector::calculateGreenConfidence(const cv::Mat &roi)
    {
        cv::Scalar mean_color = cv::mean(roi); // 使用cv::mean计算平均颜色

        // 分别计算RGB的平均值
        int sum_b = static_cast<int>(mean_color[0]);
        int sum_g = static_cast<int>(mean_color[1]);
        int sum_r = static_cast<int>(mean_color[2]);
        int pixel_count = roi.cols * roi.rows;

        // 确保分母不为0，避免精度问题
        double total = sum_b + sum_g + sum_r + 1e-5; // 防止分母为零
        double g_confidence = sum_g / total; // 绿色置信度
        return g_confidence;
    }

    Light Detector::findGreenLight(const cv::Mat &img, const cv::Mat &binary_image)
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        Light best_light; // 用来存储最有可能的光源
        double highest_confidence = 0.0; // 当前最高的置信度

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < 1.0)
                continue; // 跳过过小的轮廓

            cv::Rect minRect = cv::boundingRect(contour);

            // 确保矩形在图像范围内
            if (minRect.x < 0 || minRect.y < 0 || minRect.x + minRect.width > img.cols || minRect.y + minRect.height > img.rows)
            {
                continue;
            }
            // TODO：加入长宽比检测 但是目前不稳定，
            // double aspect_ratio = static_cast<double>(std::max(minRect.width, minRect.height)) / static_cast<double>(std::min(minRect.width, minRect.height));
            //  if (aspect_ratio > max_aspect_ratio_)
            //  {
            //      continue;
            //  }

            // 提取ROI并计算绿色置信度
            cv::Mat roi = img(minRect);
            double g_confidence = calculateGreenConfidence(roi);

            float max_value = std::max({static_cast<float>(cv::mean(roi)[2]), static_cast<float>(cv::mean(roi)[1]), static_cast<float>(cv::mean(roi)[0])});
            if (max_value == cv::mean(roi)[1] && g_confidence > highest_confidence) // 确保绿色是主导色并且置信度最高
            {
                best_light.g_confidence = g_confidence;
                best_light.box = minRect;
                best_light.center_point = (minRect.tl() + minRect.br()) * 0.5;
                highest_confidence = g_confidence;
                best_light.is_detected = true; // 标记为已检测到光源
            }
        }

        // 如果需要可视化，调用绘制函数
        drawLight(best_light);

        return best_light; // 返回最有可能的光源
    }

    void Detector::drawLight(const Light &light)
    {
        cv::rectangle(debug_image_, light.box, cv::Scalar(0, 0, 255), -1); // 绿色矩形框

        cv::circle(debug_image_, light.center_point, 5, cv::Scalar(0, 0, 255), -1); // 红色圆点

        std::string score_text = "Score: " + std::to_string(light.g_confidence);

    }

    // TODO:加入白灯检测，但是目前的绿灯跟白灯太接近
    //  bool Detector::isWhiteLight(int sum_r, int sum_g, int sum_b)
    //  {
    //     return sum_r > white_avg_r_ && sum_g > white_avg_g_ && sum_b > white_avg_b_ && std::abs(sum_r - sum_g) < white_color_tolerance_ && std::abs(sum_r - sum_b) < white_color_tolerance_ &&
    //         std::abs(sum_g - sum_b) < white_color_tolerance_;
    // }


} // namespace rm_auto_light
