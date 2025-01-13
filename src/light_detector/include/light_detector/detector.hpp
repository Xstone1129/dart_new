#pragma once

// OpenCV相关头文件
#include </usr/include/opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

// 标准库头文件
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// 项目中的自定义头文件
#include "light.hpp"


namespace rm_auto_light
{
    // Detector类定义
    class Detector
    {
    public:
        // 构造函数
        Detector(const int &bin_thres);

        // 主要检测功能
        Light detect(const cv::Mat &input);
        // 图像数据
        cv::Mat binary_img_; // 二值图像
        cv::Mat debug_image_; // 调试图像
        cv::Mat dilate_img_;

    private:
        // 图像预处理
        cv::Mat preprocessImage(const cv::Mat &input);
        double calculateGreenConfidence(const cv::Mat &roi);
        Light findGreenLight(const cv::Mat &img, const cv::Mat &binary_image);

        void drawLight(const Light &light);

        bool isWhiteLight(const int &sum_r, const int &sum_g, const int &sum_b);
        // 参数设置
        int binary_thres; // 二值化阈值
    };
} // namespace rm_auto_light
