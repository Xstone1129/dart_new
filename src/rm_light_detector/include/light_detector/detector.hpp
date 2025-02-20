#pragma once

// OpenCV相关头文件
#include </usr/include/opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
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
#include <utility>
#include <vector>

// 项目中的自定义头文件
#include "light.hpp"


namespace rm_auto_light
{
    // Detector类定义
    class Detector
    {

    public:
        struct GreenLightParams
        {
            double min_ratio;
            double max_ratio;
            double max_angle;
        };
        struct LightParams
        {
            // width / height
            double min_ratio;
            double max_ratio;
            // vertical angle
            double max_angle;
        };

        struct ArmorParams
        {
            double min_light_ratio;
            // light pairs distance
            double min_small_center_distance;
            double max_small_center_distance;
            double min_large_center_distance;
            double max_large_center_distance;
            // horizontal angle
            double max_angle;
        };

    public:
        GreenLight detect(const cv::Mat &input);
        cv::Mat binary_img_; // 二值图像
        cv::Mat debug_image_; // 调试图像
        cv::Mat dilate_img_;
        int binary_thres; // 二值化阈值
        int detect_color;
        GreenLight green_light_;
        GreenLightParams g_;
        LightParams l_;
        ArmorParams a_;
        std::vector<Light> lights_;
        std::vector<Armor> armors_;
        std::vector<GreenLight> green_lights_;
        Detector(const int &bin_thres, const int &color, const LightParams &l, const ArmorParams &a, GreenLightParams &g);
        std::pair<int, double> color_and_confidence;

    private:
        // 图像预处理
        cv::Mat preprocessImage(const cv::Mat &input);
        std::pair<int, double> findColor(const cv::Mat &img, const std::vector<cv::Point> &contour);
        GreenLight findGreenLight(const cv::Mat &img, const cv::Mat &binary_image);
        bool isLight(const Light &light);
        ArmorType isArmor(const Light &light_1, const Light &light_2);
        bool isGreenLight(const GreenLight &GreenLight);
        std::vector<Armor> matchLights(const std::vector<Light> &lights);
        bool containLight(const Light &light_1, const Light &light_2, const std::vector<Light> &lights);
        void drawResults(cv::Mat &img);
        bool isWhiteLight(const int &sum_r, const int &sum_g, const int &sum_b);
    };
} // namespace rm_auto_light
