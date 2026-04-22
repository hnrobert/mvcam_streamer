#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace mvcam_streamer
{
class MvcamStreamerNode : public rclcpp::Node
{
public:
  explicit MvcamStreamerNode(const rclcpp::NodeOptions & options) : Node("mvcam_streamer", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting MvcamStreamerNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    if (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found! Will keep scaning once a sec...");
    }

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      // RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);
    RCLCPP_INFO(this->get_logger(), "Camera opened!");

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    RCLCPP_INFO(
      this->get_logger(), "Camera info: max width = %d, max height = %d", img_info_.nWidthMax,
      img_info_.nHeightMax);

    // Init convert param
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;
    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
    compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    RCLCPP_INFO(this->get_logger(), "Camera publisher created!");

    declareParameters();

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://mvcam_streamer/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MvcamStreamerNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      // Buffer for rotated image
      std::vector<unsigned char> rotated_buffer;

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          // Set dimensions for the original image
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;

          // Resize image data buffer
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);
          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = image_msg_.data.size();

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          if (rotation_angle_ != 0) {
            // If rotation is needed, apply it
            rotated_buffer.resize(image_msg_.width * image_msg_.height * 3);
            unsigned int dst_size = rotated_buffer.size();

            if (
              rotateImage(
                image_msg_.data.data(), image_msg_.data.size(), rotated_buffer.data(), dst_size,
                rotation_angle_)) {
              // After rotation, update image data
              image_msg_.data.resize(dst_size);
              std::memcpy(image_msg_.data.data(), rotated_buffer.data(), dst_size);
            }
          }

          image_msg_.header.stamp = this->now();
          camera_info_msg_.header = image_msg_.header;

          // Update camera_info dimensions to match the image
          camera_info_msg_.width = image_msg_.width;
          camera_info_msg_.height = image_msg_.height;

          camera_pub_.publish(image_msg_, camera_info_msg_);

          if (publish_compressed_ && encodeJpeg(image_msg_, compressed_msg_.data)) {
            compressed_msg_.header = image_msg_.header;
            compressed_msg_.format = "jpeg";
            compressed_pub_->publish(compressed_msg_);
          }

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_count_ = 0;
        } else {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_count_++;
        }

        if (fail_count_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~MvcamStreamerNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "MvcamStreamerNode destroyed!");
  }

private:
  void declareParameters()
  {
    // Exposure time
    rcl_interfaces::msg::ParameterDescriptor exposure_desc;
    exposure_desc.description = "Exposure time in microseconds";
    MVCC_FLOATVALUE f_value;
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    RCLCPP_INFO(this->get_logger(), "Exposure time range: [%f, %f]", f_value.fMin, f_value.fMax);
    exposure_desc.floating_point_range.resize(1);
    exposure_desc.floating_point_range[0].from_value = f_value.fMin;
    exposure_desc.floating_point_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, exposure_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    rcl_interfaces::msg::ParameterDescriptor gain_desc;
    gain_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    RCLCPP_INFO(this->get_logger(), "Gain range: [%f, %f]", f_value.fMin, f_value.fMax);
    gain_desc.floating_point_range.resize(1);
    gain_desc.floating_point_range[0].from_value = f_value.fMin;
    gain_desc.floating_point_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, gain_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Rotation angle
    rcl_interfaces::msg::ParameterDescriptor rot_desc;
    rot_desc.description = "Image rotation angle (0=None, 1=90deg, 2=180deg, 3=270deg)";
    rot_desc.integer_range.resize(1);
    rot_desc.integer_range[0].from_value = 0;
    rot_desc.integer_range[0].to_value = 3;
    rot_desc.integer_range[0].step = 1;
    rotation_angle_ =
      static_cast<MV_IMG_ROTATION_ANGLE>(this->declare_parameter("rotation_angle", 0, rot_desc));
    RCLCPP_INFO(this->get_logger(), "Rotation angle: %d", static_cast<int>(rotation_angle_));

    // JPEG quality for compressed stream
    rcl_interfaces::msg::ParameterDescriptor jpeg_quality_desc;
    jpeg_quality_desc.description = "JPEG quality for image_raw/compressed (1-100)";
    jpeg_quality_desc.integer_range.resize(1);
    jpeg_quality_desc.integer_range[0].from_value = 1;
    jpeg_quality_desc.integer_range[0].to_value = 100;
    jpeg_quality_desc.integer_range[0].step = 1;
    jpeg_quality_ = this->declare_parameter("jpeg_quality", jpeg_quality_, jpeg_quality_desc);
    publish_compressed_ = this->declare_parameter("publish_compressed", publish_compressed_);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "rotation_angle") {
        int angle_value = param.as_int();
        if (angle_value >= 0 && angle_value <= 3) {
          rotation_angle_ = static_cast<MV_IMG_ROTATION_ANGLE>(angle_value);
          RCLCPP_INFO(this->get_logger(), "Updated rotation angle to: %d", angle_value);
        } else {
          result.successful = false;
          result.reason =
            "Invalid rotation angle value. Valid values: 0=None, 1=90deg, 2=180deg, 3=270deg";
        }
      } else if (param.get_name() == "jpeg_quality") {
        const int quality = param.as_int();
        if (quality >= 1 && quality <= 100) {
          jpeg_quality_ = quality;
        } else {
          result.successful = false;
          result.reason = "Invalid jpeg_quality value. Valid range: 1-100";
        }
      } else if (param.get_name() == "publish_compressed") {
        publish_compressed_ = param.as_bool();
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  bool rotateImage(
    const unsigned char * pSrcData, unsigned int srcDataLen, unsigned char * pDstData,
    unsigned int & dstDataLen, MV_IMG_ROTATION_ANGLE rotation)
  {
    if (rotation == 0) {
      // No rotation needed
      if (srcDataLen <= dstDataLen) {
        std::memcpy(pDstData, pSrcData, srcDataLen);
        dstDataLen = srcDataLen;
        return true;
      } else {
        return false;
      }
    }

    MV_CC_ROTATE_IMAGE_PARAM rotate_param;
    memset(&rotate_param, 0, sizeof(MV_CC_ROTATE_IMAGE_PARAM));
    rotate_param.enPixelType = convert_param_.enDstPixelType;
    rotate_param.nHeight = image_msg_.height;
    rotate_param.nWidth = image_msg_.width;
    rotate_param.pSrcData = const_cast<unsigned char *>(pSrcData);
    rotate_param.nSrcDataLen = srcDataLen;
    rotate_param.pDstBuf = pDstData;
    rotate_param.nDstBufSize = dstDataLen;
    rotate_param.enRotationAngle = rotation;

    int status = MV_CC_RotateImage(camera_handle_, &rotate_param);
    if (status == MV_OK) {
      dstDataLen = rotate_param.nDstBufLen;

      // Update width and height if rotating 90 or 270 degrees
      if (rotation == MV_IMAGE_ROTATE_90 || rotation == MV_IMAGE_ROTATE_270) {
        std::swap(image_msg_.width, image_msg_.height);
        image_msg_.step = image_msg_.width * 3;
      }

      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to rotate image, error code: %d", status);
      return false;
    }
  }

  bool encodeJpeg(const sensor_msgs::msg::Image & image, std::vector<uint8_t> & output)
  {
    if (image.encoding != "rgb8") {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Unsupported image encoding for JPEG conversion: %s", image.encoding.c_str());
      return false;
    }

    cv::Mat rgb(static_cast<int>(image.height), static_cast<int>(image.width), CV_8UC3);
    std::memcpy(rgb.data, image.data.data(), image.data.size());

    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

    std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    return cv::imencode(".jpg", bgr, output, params);
  }

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CompressedImage compressed_msg_;

  image_transport::CameraPublisher camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;

  int nRet = MV_OK;
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_count_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  MV_IMG_ROTATION_ANGLE rotation_angle_ = MV_IMAGE_ROTATE_180;
  int jpeg_quality_ = 85;
  bool publish_compressed_ = true;
};
}  // namespace mvcam_streamer

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mvcam_streamer::MvcamStreamerNode)
