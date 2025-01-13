#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std;

class VideoPublisher : public rclcpp::Node {
 public:
  explicit VideoPublisher(const std::string &video_path)
      : Node("video_publisher"), video_path_(video_path) {
    bool use_sensor_data_qos =
        this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                   : rmw_qos_profile_default;
    camera_pub_ =
        image_transport::create_camera_publisher(this, "image_raw", qos);

    cv::VideoCapture cap(video_path_);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
      return;
    }

    string camera_name_ = this->declare_parameter("camera_name", "mv_camera");
    camera_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                                 camera_name_);
    auto camera_info_url = this->declare_parameter(
        "camera_info_url",
        "package://mindvision_camera/config/camera_info.yaml");

    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                  camera_info_url.c_str());
    }

    // 主循环，用于视频流读取和发布
    cv::Mat frame;
    rclcpp::Rate loop_rate(30);  // 设定发布频率为30Hz

    while (rclcpp::ok()) {
      if (cap.read(frame)) {
        publishImage(frame);
      } else {
        cap.release();          // 释放 VideoCapture 对象
        cap.open(video_path_);  // 重新打开视频文件
        continue;               // 继续下一次循环
      }

      // 处理ROS回调，允许其他的ROS操作
      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();  // 控制循环频率
    }

    cap.release();  // 循环结束后释放资源
  }

 private:
  void publishImage(const cv::Mat &image) {
    cv::Mat resize_img;
    cv::resize(image, resize_img, cv::Size(960, 590));  // 调整图像大小
    cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8",
                                resize_img);  // 使用CvImage转换格式
    sensor_msgs::msg::Image::SharedPtr ros_image =
        cv_image.toImageMsg();                            // 转换为ROS消息
    ros_image->header.frame_id = "camera_optical_frame";  // 设置ROS消息头
    camera_pub_.publish(*ros_image, camera_info_msg_);    // 发布图像
  }

  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string video_path_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // 创建并启动视频发布节点，传入视频路径
  VideoPublisher video_publisher(
      "/home/xu/kunkun_final/src/img_test/the_blue.mp4");

  rclcpp::shutdown();  // 关闭ROS节点
  return 0;
}
