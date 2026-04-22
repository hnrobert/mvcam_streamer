#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace hik_camera
{
class MjpegStreamServer : public rclcpp::Node
{
public:
  MjpegStreamServer() : Node("mjpeg_stream_server")
  {
    bind_address_ = this->declare_parameter("bind_address", "127.0.0.1");
    port_ = this->declare_parameter("port", 8080);
    stream_path_ = this->declare_parameter("stream_path", "/stream.mjpg");
    input_topic_ = this->declare_parameter("input_topic", "/image_raw/compressed");

    if (port_ <= 0 || port_ > 65535) {
      RCLCPP_WARN(this->get_logger(), "Invalid port %d, fallback to 8080", port_);
      port_ = 8080;
    }

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
        if (
          msg->format.find("jpeg") == std::string::npos &&
          msg->format.find("jpg") == std::string::npos) {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Input compressed format is '%s', expected jpeg/jpg", msg->format.c_str());
          return;
        }

        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          latest_jpeg_ = msg->data;
        }
        frame_cv_.notify_all();
      });

    server_thread_ = std::thread([this]() { this->runServer(); });

    RCLCPP_INFO(
      this->get_logger(), "MJPEG stream server started at http://%s:%d%s (topic: %s)",
      bind_address_.c_str(), port_, stream_path_.c_str(), input_topic_.c_str());
  }

  ~MjpegStreamServer() override
  {
    running_ = false;
    if (listen_fd_ >= 0) {
      close(listen_fd_);
      listen_fd_ = -1;
    }

    frame_cv_.notify_all();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  static bool sendAll(int fd, const uint8_t * data, size_t size)
  {
    size_t sent = 0;
    while (sent < size) {
      const ssize_t n = send(fd, data + sent, size - sent, 0);
      if (n <= 0) {
        return false;
      }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

  static bool sendAll(int fd, const std::string & data)
  {
    return sendAll(fd, reinterpret_cast<const uint8_t *>(data.data()), data.size());
  }

  void runServer()
  {
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
      return;
    }

    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid bind address: %s", bind_address_.c_str());
      return;
    }

    if (bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed on %s:%d", bind_address_.c_str(), port_);
      return;
    }

    if (listen(listen_fd_, 8) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Listen failed");
      return;
    }

    while (running_ && rclcpp::ok()) {
      sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(listen_fd_, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (running_) {
          RCLCPP_WARN(this->get_logger(), "Accept failed");
        }
        continue;
      }

#ifdef __APPLE__
      int no_sigpipe = 1;
      setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe));
#endif

      handleClient(client_fd);
      close(client_fd);
    }
  }

  void handleClient(int client_fd)
  {
    char request_buf[1024];
    const ssize_t request_len = recv(client_fd, request_buf, sizeof(request_buf) - 1, 0);
    if (request_len <= 0) {
      return;
    }
    request_buf[request_len] = '\0';

    std::string req(request_buf);
    const std::string expected = "GET " + stream_path_ + " ";
    if (req.find(expected) != 0) {
      const std::string body =
        "<html><body><h3>MJPEG Server</h3><p>Open " + stream_path_ + "</p></body></html>";
      std::ostringstream oss;
      oss << "HTTP/1.1 200 OK\r\n"
          << "Content-Type: text/html\r\n"
          << "Content-Length: " << body.size() << "\r\n\r\n"
          << body;
      sendAll(client_fd, oss.str());
      return;
    }

    const std::string header =
      "HTTP/1.1 200 OK\r\n"
      "Connection: close\r\n"
      "Cache-Control: no-cache\r\n"
      "Pragma: no-cache\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
    if (!sendAll(client_fd, header)) {
      return;
    }

    while (running_ && rclcpp::ok()) {
      std::vector<uint8_t> frame;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
          return !latest_jpeg_.empty() || !running_ || !rclcpp::ok();
        });
        if (!running_ || !rclcpp::ok()) {
          break;
        }
        frame = latest_jpeg_;
      }

      if (frame.empty()) {
        continue;
      }

      std::ostringstream oss;
      oss << "--frame\r\n"
          << "Content-Type: image/jpeg\r\n"
          << "Content-Length: " << frame.size() << "\r\n\r\n";

      if (!sendAll(client_fd, oss.str())) {
        break;
      }
      if (!sendAll(client_fd, frame.data(), frame.size())) {
        break;
      }
      if (!sendAll(client_fd, "\r\n")) {
        break;
      }
    }
  }

  std::string bind_address_;
  int port_;
  std::string stream_path_;
  std::string input_topic_;

  std::atomic<bool> running_{true};
  int listen_fd_{-1};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  std::mutex frame_mutex_;
  std::condition_variable frame_cv_;
  std::vector<uint8_t> latest_jpeg_;

  std::thread server_thread_;
};
}  // namespace hik_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hik_camera::MjpegStreamServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
