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

namespace rm_auto_light
{

    Detector::Detector(const int &bin_thres) : binary_thres(bin_thres)
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
    Light Detector::detect(const cv::Mat &input)
    {
        debug_image_ = input;
        binary_img_ = preprocessImage(input);
        return findGreenLight(input, binary_img_);
    }

    // 计算给定区域的绿色置信度
    double Detector::calculateGreenConfidence(const cv::Mat &img, const std::vector<cv::Point> &contour)
    {
        double sum_b = 0, sum_g = 0, sum_r = 0;
        for (const cv::Point &pt : contour)
        {
            cv::Vec3b color = img.at<cv::Vec3b>(pt);
            sum_r += color[2];
            sum_g += color[1];
            sum_b += color[0];
        }


        double g_confidence = sum_g / (sum_b + sum_g + sum_r); // 绿色置信度
        // std::cout << "sum_r: " << sum_r << std::endl;
        // std::cout << "sum_g: " << sum_g << std::endl;
        // std::cout << "sum_b: " << sum_b << std::endl;
        // std::cout << "g_confidence: " << g_confidence << std::endl;
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
            if (area < 10.0)
                continue; // 跳过过小的轮廓

            cv::Rect minRect = cv::boundingRect(contour);
            cv::Point2f light_center(0, 0);
            float light_radius = 0;

            cv::minEnclosingCircle(contour, light_center, light_radius);

            double circle_area = CV_PI * light_radius * light_radius;
            double area_ratio = area / circle_area * 100.0;
            // 确保矩形在图像范围内
            if (minRect.x < 0 || minRect.y < 0 || minRect.x + minRect.width > img.cols || minRect.y + minRect.height > img.rows)
            {
                continue;
            }
            // TODO：加入长宽比检测 但是目前不稳定，
            // double aspect_ratio = static_cast<double>(std::max(minRect.width, minRect.height)) / static_cast<double>(std::min(minRect.width,
            // minRect.height));
            //  if (aspect_ratio > max_aspect_ratio_)
            //  {
            //      continue;
            //  }

            // 提取ROI并计算绿色置信度
            cv::Mat roi = img(minRect);
            // std::cout<<"1111"<<std::endl;
            double g_confidence = calculateGreenConfidence(img, contour)*100;

            if (g_confidence > highest_confidence&&g_confidence>=40.0) // 确保绿色是主导色并且置信度最高
            {
                best_light.g_confidence = g_confidence;
                best_light.box = minRect;

                cv::Point2f center;
                cv::Moments moments = cv::moments(contour);
                center = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00);
                // best_light.center_point = (minRect.tl() + minRect.br()) * 0.5;
                best_light.center_point = center;
                highest_confidence = g_confidence;
                best_light.is_detected = true; // 标记为已检测到光源
            }
        }

        std::cout<<"best_light.g_confidence: "<<best_light.g_confidence<<std::endl;
        // 如果需要可视化，调用绘制函数
        drawLight(best_light);

        return best_light; // 返回最有可能的光源
    }

    void Detector::drawLight(const Light &light)
    {
        // cv::rectangle(debug_image_, light.box, cv::Scalar(0, 0, 255), -1); // 绿色矩形框
        cv::circle(debug_image_, light.center_point, 5, cv::Scalar(0, 0, 255), -1); // 红色圆点

        std::string score_text = "Score: " + std::to_string(light.g_confidence);
    }

    // TODO:加入白灯检测，但是目前的绿灯跟白灯太接近
    //  bool Detector::isWhiteLight(int sum_r, int sum_g, int sum_b)
    //  {
    //     return sum_r > white_avg_r_ && sum_g > white_avg_g_ && sum_b > white_avg_b_ && std::abs(sum_r - sum_g) < white_color_tolerance_ &&
    //     std::abs(sum_r - sum_b) < white_color_tolerance_ &&
    //         std::abs(sum_g - sum_b) < white_color_tolerance_;
    // }


} // namespace rm_auto_light
