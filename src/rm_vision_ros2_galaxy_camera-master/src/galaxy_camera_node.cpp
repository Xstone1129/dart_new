#include "GxIAPI.h"
#include "DxImageProc.h"
#include <condition_variable>
#include <mutex>
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#define GX_SUCCESS(X) (X == GX_STATUS_SUCCESS)

namespace galaxy_camera
{
class GalaxyCameraNode : public rclcpp::Node
{
public:
  explicit GalaxyCameraNode(const rclcpp::NodeOptions & options) : Node("galaxy_camera", options)  
  {
    GX_STATUS status;
    RCLCPP_INFO(this->get_logger(), "Starting GalaxyCameraNode!");

    // Init lib
    do {
      status = GXInitLib();
      if (!GX_SUCCESS(status)) {
        RCLCPP_FATAL(this->get_logger(), "Init GxIAPI failed, code = %x!", status);
        exit(status);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    } while (!GX_SUCCESS(status));

    // Constantly try opening one camera
    while (true) {
      uint32_t device_count = 0;
      status = GXUpdateDeviceList(&device_count, 100);
      if (device_count < 1) {
        RCLCPP_WARN(this->get_logger(), "No camera found. device_count = %d", device_count);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      status = GXOpenDeviceByIndex(1, &camera_handle_);
      if (!GX_SUCCESS(status)) {
        RCLCPP_ERROR(this->get_logger(), "Can not open camera, status = %d", status);
      } else {
        break;
      }
    };

    // Get camera infomation
    GXGetInt(camera_handle_, GX_INT_WIDTH, &img_info_.nWidthValue);
    GXGetInt(camera_handle_, GX_INT_WIDTH_MAX, &img_info_.nWidthMax);
    GXGetInt(camera_handle_, GX_INT_HEIGHT, &img_info_.nHeightValue);
    GXGetInt(camera_handle_, GX_INT_HEIGHT_MAX, &img_info_.nHeightMax);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    // std::cout<<"======="<<img_info_.nWidthValue<<std::endl;
    // std::cout<<"======="<<img_info_.nHeightValue<<std::endl;


//1.11change:这个从true改成false了 不然接收不到画面
    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", false);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    declareParameters();

    GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_START);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://galaxy_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GalaxyCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      GX_FRAME_DATA bayer_frame {};
      GX_STATUS status;
      std::vector<char> bayer_buffer_holder;

      // Initialize frame
      int64_t payloadSize;
      GXGetInt(camera_handle_, GX_INT_PAYLOAD_SIZE, &payloadSize);
      bayer_buffer_holder.reserve(payloadSize);
      bayer_frame.pImgBuf = bayer_buffer_holder.data();

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        // Fetch image
        status = GXGetImage(camera_handle_, &bayer_frame, 500);
        
        if (GX_SUCCESS(status)) {
          DX_PIXEL_COLOR_FILTER bayer_type;
          switch (bayer_frame.nPixelFormat) {
          	case GX_PIXEL_FORMAT_BAYER_GR8: bayer_type = BAYERGR; break;
            case GX_PIXEL_FORMAT_BAYER_RG8: bayer_type = BAYERRG; break;
            case GX_PIXEL_FORMAT_BAYER_GB8: bayer_type = BAYERGB; break;
            case GX_PIXEL_FORMAT_BAYER_BG8: bayer_type = BAYERBG; break;
            default: RCLCPP_FATAL(this->get_logger(), "Unsupported Bayer layout: %d!", bayer_frame.nPixelFormat); return;
          }

          image_msg_.header.stamp = this->now();
          image_msg_.height = bayer_frame.nHeight;
          image_msg_.width = bayer_frame.nWidth;
          image_msg_.step = bayer_frame.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

    // std::cout<<"???????????????"<<image_msg_.width<<std::endl;
    // std::cout<<"?????????????"<<image_msg_.height<<std::endl;

          status = DxRaw8toRGB24(bayer_frame.pImgBuf, image_msg_.data.data(),
                                 bayer_frame.nWidth, bayer_frame.nHeight,
                                 RAW2RGB_NEIGHBOUR, bayer_type, false);

          if (!GX_SUCCESS(status)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert Bayer to RGB, status = %d", status);
            continue;
          }

          RCLCPP_DEBUG(this->get_logger(), "Publish image: %dx%d", bayer_frame.nWidth, bayer_frame.nHeight);

          camera_info_msg_.header = image_msg_.header;
          camera_pub_.publish(image_msg_, camera_info_msg_);

          fail_conut_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed, status = %d", status);
          GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_STOP);
          GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_START);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};

    RCLCPP_INFO(this->get_logger(), "GalaxyCamera initialized");
  }

  ~GalaxyCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      GXSendCommand(camera_handle_, GX_COMMAND_ACQUISITION_STOP);
      GXCloseDevice(camera_handle_);
    }
    GXCloseLib();
    RCLCPP_INFO(this->get_logger(), "GalaxyCameraNode destroyed!");
  }

private:
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    double f_value;
    GX_FLOAT_RANGE f_range;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    GXGetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &f_value);
    GXGetFloatRange(camera_handle_, GX_FLOAT_EXPOSURE_TIME, &f_range);
    param_desc.integer_range[0].from_value = f_range.dMin;
    param_desc.integer_range[0].to_value = f_range.dMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    GXGetFloat(camera_handle_, GX_FLOAT_GAIN, &f_value);
    GXGetFloatRange(camera_handle_, GX_FLOAT_GAIN, &f_range);
    param_desc.integer_range[0].from_value = f_range.dMin;
    param_desc.integer_range[0].to_value = f_range.dMax;
    double gain = this->declare_parameter("gain", f_value, param_desc);
    GXSetFloat(camera_handle_, GX_FLOAT_GAIN, gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    GX_STATUS status;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        status = GXSetFloat(camera_handle_, GX_FLOAT_EXPOSURE_TIME, param.as_int());
        if (GX_SUCCESS(status)) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        status = GXSetFloat(camera_handle_, GX_FLOAT_GAIN, param.as_double());
        if (GX_SUCCESS(status)) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  GX_DEV_HANDLE camera_handle_;
  struct {
    int64_t nWidthValue, nWidthMax;
    int64_t nHeightValue, nHeightMax;
  } img_info_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace galaxy_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(galaxy_camera::GalaxyCameraNode)
