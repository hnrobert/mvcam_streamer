#include <cstdio>
#include <csignal>
#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace mvcam_streamer
{
class H264RtspStreamServer : public rclcpp::Node
{
public:
  H264RtspStreamServer() : Node("h264_rtsp_stream_server")
  {
    ffmpeg_bin_ = this->declare_parameter("ffmpeg_bin", "ffmpeg");
    bind_address_ = this->declare_parameter("bind_address", "127.0.0.1");
    port_ = this->declare_parameter("port", 8554);
    stream_path_ = this->declare_parameter("stream_path", "/stream");
    input_topic_ = this->declare_parameter("input_topic", "/image_raw/compressed");
    output_mode_ = this->declare_parameter("output_mode", "udp");

    bitrate_kbps_ = this->declare_parameter("bitrate_kbps", 2500);
    gop_ = this->declare_parameter("gop", 30);
    listen_mode_ = this->declare_parameter("listen_mode", true);
    restart_on_failure_ = this->declare_parameter("restart_on_failure", false);
    max_restart_attempts_ = this->declare_parameter("max_restart_attempts", 3);
    int retry_interval_ms = this->declare_parameter("retry_interval_ms", 10000);
    if (retry_interval_ms < 1000) {
      retry_interval_ms = 1000;
    }
    retry_interval_ = std::chrono::milliseconds(retry_interval_ms);

    normalizeParameters();

    if (!startFfmpegProcess()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start FFmpeg RTSP process");
      if (restart_on_failure_) {
        next_retry_time_ = std::chrono::steady_clock::now() + retry_interval_;
      } else {
        streaming_disabled_ = true;
      }
    }

    reconnect_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      ensureFfmpegProcess();
    });

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        const bool is_jpeg =
          msg->format.find("jpeg") != std::string::npos ||
          msg->format.find("jpg") != std::string::npos;
        if (!is_jpeg) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Input compressed format is '%s', expected jpeg/jpg", msg->format.c_str());
          return;
        }

        writeFrameToFfmpeg(msg->data);
      });

    RCLCPP_INFO(
      this->get_logger(),
      "MPEG4 stream server started (%s): %s (topic: %s, bitrate: %dkbps, gop: %d)",
      output_mode_.c_str(), makeOutputUrl().c_str(), input_topic_.c_str(), bitrate_kbps_, gop_);
  }

  ~H264RtspStreamServer() override
  {
    stopFfmpegProcess();
  }

private:
  void normalizeParameters()
  {
    if (port_ <= 0 || port_ > 65535) {
      RCLCPP_WARN(this->get_logger(), "Invalid port %d, fallback to 8554", port_);
      port_ = 8554;
    }
    if (bitrate_kbps_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid bitrate_kbps %d, fallback to 2500", bitrate_kbps_);
      bitrate_kbps_ = 2500;
    }
    if (gop_ <= 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid gop %d, fallback to 30", gop_);
      gop_ = 30;
    }
    if (stream_path_.empty() || stream_path_[0] != '/') {
      stream_path_ = "/" + stream_path_;
    }
    if (output_mode_ != "udp" && output_mode_ != "rtsp") {
      RCLCPP_WARN(this->get_logger(), "Invalid output_mode '%s', fallback to udp", output_mode_.c_str());
      output_mode_ = "udp";
    }
  }

  std::string makeOutputUrl() const
  {
    if (output_mode_ == "udp") {
      std::ostringstream url;
      url << "udp://" << bind_address_ << ":" << port_ << "?pkt_size=1316";
      return url.str();
    }

    std::ostringstream url;
    url << "rtsp://" << bind_address_ << ":" << port_ << stream_path_;
    if (listen_mode_) {
      url << "?listen=1";
    }
    return url.str();
  }

  std::string makeFfmpegCommand() const
  {
    std::ostringstream cmd;
    const std::string output_url = makeOutputUrl();

    cmd << ffmpeg_bin_;
    cmd << " -hide_banner -loglevel warning -nostdin";
    cmd << " -f mjpeg -i -";
    cmd << " -an -c:v mpeg4";
    cmd << " -pix_fmt yuv420p";
    cmd << " -g " << gop_;
    cmd << " -keyint_min " << gop_;
    cmd << " -b:v " << bitrate_kbps_ << "k";
    if (output_mode_ == "udp") {
      cmd << " -f mpegts";
    } else {
      cmd << " -f rtsp -rtsp_transport tcp -muxdelay 0.1";
    }
    cmd << " \"" << output_url << "\"";
    return cmd.str();
  }

  bool startFfmpegProcess()
  {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (ffmpeg_pipe_ != nullptr) {
      return true;
    }

    const std::string command = makeFfmpegCommand();
    RCLCPP_INFO(this->get_logger(), "Starting FFmpeg: %s", command.c_str());

    ffmpeg_pipe_ = ::popen(command.c_str(), "w");
    return ffmpeg_pipe_ != nullptr;
  }

  void stopFfmpegProcess()
  {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (ffmpeg_pipe_ != nullptr) {
      ::pclose(ffmpeg_pipe_);
      ffmpeg_pipe_ = nullptr;
    }
  }

  void ensureFfmpegProcess()
  {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (streaming_disabled_) {
      return;
    }

    if (ffmpeg_pipe_ != nullptr) {
      return;
    }

    if (!restart_on_failure_) {
      return;
    }

    if (restart_attempts_ >= max_restart_attempts_) {
      streaming_disabled_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "FFmpeg reached max restart attempts (%d). Streaming disabled. "
        "If using an external RTSP server, check its status and restart this node.",
        max_restart_attempts_);
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now < next_retry_time_) {
      return;
    }

    const std::string command = makeFfmpegCommand();
    RCLCPP_INFO(this->get_logger(), "Restarting FFmpeg: %s", command.c_str());
    ++restart_attempts_;
    ffmpeg_pipe_ = ::popen(command.c_str(), "w");
    if (ffmpeg_pipe_ == nullptr) {
      next_retry_time_ = now + retry_interval_;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "FFmpeg restart failed, will retry in %ld ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(retry_interval_).count());
    }
  }

  void restartFfmpegProcessLocked()
  {
    if (ffmpeg_pipe_ != nullptr) {
      ::pclose(ffmpeg_pipe_);
      ffmpeg_pipe_ = nullptr;
    }

    if (!restart_on_failure_) {
      streaming_disabled_ = true;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "FFmpeg exited and restart_on_failure=false. Streaming disabled to avoid restart storm.");
      return;
    }

    next_retry_time_ = std::chrono::steady_clock::now() + retry_interval_;
  }

  void writeFrameToFfmpeg(const std::vector<uint8_t> & frame)
  {
    std::lock_guard<std::mutex> lock(pipe_mutex_);
    if (ffmpeg_pipe_ == nullptr) {
      return;
    }

    const size_t written = std::fwrite(frame.data(), 1, frame.size(), ffmpeg_pipe_);
    if (written != frame.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "FFmpeg write failed (%zu/%zu), will retry FFmpeg startup with backoff", written,
        frame.size());
      restartFfmpegProcessLocked();
      return;
    }

    std::fflush(ffmpeg_pipe_);
  }

  std::string ffmpeg_bin_;
  std::string bind_address_;
  int port_;
  std::string stream_path_;
  std::string input_topic_;
  std::string output_mode_;
  int bitrate_kbps_;
  int gop_;
  bool listen_mode_;
  bool restart_on_failure_;
  int max_restart_attempts_;
  int restart_attempts_{0};
  bool streaming_disabled_{false};

  FILE * ffmpeg_pipe_{nullptr};
  std::mutex pipe_mutex_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  std::chrono::milliseconds retry_interval_{2000};
  std::chrono::steady_clock::time_point next_retry_time_{std::chrono::steady_clock::now()};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
};
}  // namespace mvcam_streamer

int main(int argc, char ** argv)
{
  // Prevent process termination if FFmpeg exits and pipe writes hit EPIPE.
  std::signal(SIGPIPE, SIG_IGN);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<mvcam_streamer::H264RtspStreamServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
