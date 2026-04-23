#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>

namespace mvcam_streamer
{
namespace
{
struct HttpRequest
{
  std::string method;
  std::string target;  // includes optional query
  std::string path;    // without query
  std::string query;   // without leading '?'
  std::vector<std::pair<std::string, std::string>> headers;
  std::string body;
};

static std::string trim(const std::string & s)
{
  size_t b = 0;
  while (b < s.size() && (s[b] == ' ' || s[b] == '\t' || s[b] == '\r' || s[b] == '\n')) {
    ++b;
  }
  size_t e = s.size();
  while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r' || s[e - 1] == '\n')) {
    --e;
  }
  return s.substr(b, e - b);
}

static std::string toLower(std::string s)
{
  for (auto & c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  return s;
}

static std::optional<std::string> headerValue(const HttpRequest & req, const std::string & key)
{
  const std::string key_l = toLower(key);
  for (const auto & kv : req.headers) {
    if (toLower(kv.first) == key_l) {
      return kv.second;
    }
  }
  return std::nullopt;
}

static bool sendAll(int fd, const uint8_t * data, size_t size)
{
  size_t sent = 0;
  while (sent < size) {
    const ssize_t n = ::send(fd, data + sent, size - sent, MSG_NOSIGNAL);
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

static void splitTarget(const std::string & target, std::string * path, std::string * query)
{
  const auto pos = target.find('?');
  if (pos == std::string::npos) {
    *path = target;
    query->clear();
    return;
  }
  *path = target.substr(0, pos);
  *query = target.substr(pos + 1);
}

static std::optional<std::string> queryParam(const std::string & query, const std::string & key)
{
  // Very small parser: key=value&key2=value2 (no decoding)
  size_t start = 0;
  while (start < query.size()) {
    const size_t amp = query.find('&', start);
    const size_t end = (amp == std::string::npos) ? query.size() : amp;
    const size_t eq = query.find('=', start);
    if (eq != std::string::npos && eq < end) {
      const std::string k = query.substr(start, eq - start);
      if (k == key) {
        return query.substr(eq + 1, end - (eq + 1));
      }
    }
    if (amp == std::string::npos) {
      break;
    }
    start = amp + 1;
  }
  return std::nullopt;
}

static bool recvToBuffer(int fd, std::string * buf)
{
  char tmp[4096];
  const ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
  if (n <= 0) {
    return false;
  }
  buf->append(tmp, static_cast<size_t>(n));
  return true;
}

static bool parseHttpRequest(int fd, HttpRequest * out)
{
  std::string buf;
  buf.reserve(8192);

  // Read until headers complete
  while (true) {
    const auto header_end = buf.find("\r\n\r\n");
    if (header_end != std::string::npos) {
      const size_t headers_len = header_end + 4;
      const std::string header_part = buf.substr(0, header_end);

      std::istringstream iss(header_part);
      std::string line;
      if (!std::getline(iss, line)) {
        return false;
      }
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      {
        std::istringstream lss(line);
        if (!(lss >> out->method >> out->target)) {
          return false;
        }
      }
      splitTarget(out->target, &out->path, &out->query);

      out->headers.clear();
      while (std::getline(iss, line)) {
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
          continue;
        }
        const std::string k = trim(line.substr(0, colon));
        const std::string v = trim(line.substr(colon + 1));
        if (!k.empty()) {
          out->headers.emplace_back(k, v);
        }
      }

      size_t content_length = 0;
      if (const auto cl = headerValue(*out, "Content-Length")) {
        try {
          content_length = static_cast<size_t>(std::stoul(*cl));
        } catch (...) {
          content_length = 0;
        }
      }

      // Body may already be partially read
      out->body.clear();
      const std::string already = buf.substr(headers_len);
      if (already.size() >= content_length) {
        out->body.assign(already.data(), content_length);
        return true;
      }
      out->body = already;

      while (out->body.size() < content_length) {
        if (!recvToBuffer(fd, &buf)) {
          return false;
        }
        const std::string extra = buf.substr(headers_len + out->body.size());
        out->body.append(extra);
        if (out->body.size() > content_length) {
          out->body.resize(content_length);
        }
      }

      return true;
    }

    if (!recvToBuffer(fd, &buf)) {
      return false;
    }

    // Avoid unlimited growth
    if (buf.size() > 1024 * 1024) {
      return false;
    }
  }
}

static std::string httpResponse(
  int code, const std::string & reason, const std::string & content_type, const std::string & body)
{
  std::ostringstream oss;
  oss << "HTTP/1.1 " << code << ' ' << reason << "\r\n";
  oss << "Connection: close\r\n";
  oss << "Cache-Control: no-cache\r\n";
  oss << "Pragma: no-cache\r\n";
  oss << "Content-Type: " << content_type << "\r\n";
  oss << "Content-Length: " << body.size() << "\r\n\r\n";
  oss << body;
  return oss.str();
}

static std::string httpNoContent(int code, const std::string & reason)
{
  std::ostringstream oss;
  oss << "HTTP/1.1 " << code << ' ' << reason << "\r\n";
  oss << "Connection: close\r\n";
  oss << "Content-Length: 0\r\n\r\n";
  return oss.str();
}

static std::string indexHtml(const std::string & base_path)
{
  // Minimal single-page WebRTC viewer. Uses POST /offer and polling /candidates.
  std::ostringstream oss;
  oss << "<!doctype html><html><head><meta charset=\"utf-8\">"
      << "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
      << "<title>WebRTC Preview</title>"
      << "<style>body{margin:0;background:#111;color:#eee;font-family:sans-serif}"
      << "header{padding:12px 16px;font-size:14px;color:#bbb}"
      << "main{display:flex;justify-content:center;align-items:center;height:calc(100vh - 44px)}"
      << "video{max-width:100vw;max-height:calc(100vh - 44px);background:#000}"
      << "</style></head><body>"
      << "<header>WebRTC Preview: " << base_path << "</header>"
      << "<main><video id=\"v\" autoplay playsinline controls muted></video></main>"
      << "<script>\n"
      << "(async ()=>{\n"
      << "  const base='" << base_path << "';\n"
      << "  const video=document.getElementById('v');\n"
      << "  const pc=new RTCPeerConnection();\n"
      << "  pc.addTransceiver('video',{direction:'recvonly'});\n"
      << "  pc.ontrack=(ev)=>{ if(ev.streams && ev.streams[0]) video.srcObject=ev.streams[0]; };\n"
      << "  pc.onicecandidate=async (ev)=>{\n"
      << "    if(!ev.candidate) return;\n"
      << "    const m=ev.candidate.sdpMLineIndex;\n"
      << "    const c=ev.candidate.candidate;\n"
      << "    try{ await fetch(base+'/candidate?mline='+encodeURIComponent(m),{method:'POST',headers:{'Content-Type':'text/plain'},body:c}); }catch(e){}\n"
      << "  };\n"
      << "  const offer=await pc.createOffer();\n"
      << "  await pc.setLocalDescription(offer);\n"
      << "  const resp=await fetch(base+'/offer',{method:'POST',headers:{'Content-Type':'application/sdp'},body:pc.localDescription.sdp});\n"
      << "  if(!resp.ok){ throw new Error('offer failed'); }\n"
      << "  const answerSdp=await resp.text();\n"
      << "  await pc.setRemoteDescription({type:'answer',sdp:answerSdp});\n"
      << "  let poll=true;\n"
      << "  const pollFn=async ()=>{\n"
      << "    if(!poll) return;\n"
      << "    try{\n"
      << "      const r=await fetch(base+'/candidates');\n"
      << "      if(r.ok){\n"
      << "        const t=await r.text();\n"
      << "        const lines=t.split('\\n').filter(x=>x.trim().length);\n"
      << "        for(const line of lines){\n"
      << "          const p=line.indexOf('|');\n"
      << "          if(p<0) continue;\n"
      << "          const m=parseInt(line.slice(0,p),10);\n"
      << "          const c=line.slice(p+1);\n"
      << "          try{ await pc.addIceCandidate({sdpMLineIndex:m,candidate:c}); }catch(e){}\n"
      << "        }\n"
      << "      }\n"
      << "    }catch(e){}\n"
      << "    setTimeout(pollFn, 300);\n"
      << "  };\n"
      << "  pollFn();\n"
      << "  pc.onconnectionstatechange=()=>{\n"
      << "    if(pc.connectionState==='failed' || pc.connectionState==='closed'){ poll=false; }\n"
      << "  };\n"
      << "})();\n"
      << "</script></body></html>";
  return oss.str();
}

struct IceCandidate
{
  int mline_index;
  std::string candidate;
};

static std::string sdpMessageToText(const GstSDPMessage * sdp)
{
  gchar * txt = gst_sdp_message_as_text(sdp);
  if (txt == nullptr) {
    return std::string();
  }
  std::string out(txt);
  g_free(txt);
  return out;
}

static std::optional<GstWebRTCSessionDescription *> parseOfferSdp(const std::string & sdp_text)
{
  GstSDPMessage * sdp = nullptr;
  if (gst_sdp_message_new(&sdp) != GST_SDP_OK || sdp == nullptr) {
    return std::nullopt;
  }

  const GstSDPResult res = gst_sdp_message_parse_buffer(
    reinterpret_cast<const guint8 *>(sdp_text.data()), static_cast<guint>(sdp_text.size()), sdp);
  if (res != GST_SDP_OK) {
    gst_sdp_message_free(sdp);
    return std::nullopt;
  }

  return gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_OFFER, sdp);
}

static GstPad * requestWebrtcSinkPad(GstElement * webrtcbin)
{
  if (webrtcbin == nullptr) {
    return nullptr;
  }

  GstCaps * rtp_caps = gst_caps_new_simple(
    "application/x-rtp",
    "media", G_TYPE_STRING, "video",
    "encoding-name", G_TYPE_STRING, "VP8",
    "payload", G_TYPE_INT, 96,
    "clock-rate", G_TYPE_INT, 90000,
    nullptr);

  // First try modern convenience API.
  GstPad * pad = gst_element_request_pad_simple(webrtcbin, "sink_%u");
  if (pad != nullptr) {
    gst_caps_unref(rtp_caps);
    return pad;
  }

  // Fallback for versions/plugins where request_pad_simple fails unexpectedly.
  GstElementClass * klass = GST_ELEMENT_GET_CLASS(webrtcbin);
  if (klass == nullptr) {
    gst_caps_unref(rtp_caps);
    return nullptr;
  }

  GstPadTemplate * templ = gst_element_class_get_pad_template(klass, "sink_%u");
  if (templ == nullptr) {
    gst_caps_unref(rtp_caps);
    return nullptr;
  }

  pad = gst_element_request_pad(webrtcbin, templ, nullptr, rtp_caps);
  gst_caps_unref(rtp_caps);
  return pad;
}

static GstCaps * makeVp8RtpCaps()
{
  return gst_caps_new_simple(
    "application/x-rtp",
    "media", G_TYPE_STRING, "video",
    "encoding-name", G_TYPE_STRING, "VP8",
    "payload", G_TYPE_INT, 96,
    "clock-rate", G_TYPE_INT, 90000,
    nullptr);
}

}  // namespace

class WebrtcStreamServer : public rclcpp::Node
{
public:
  WebrtcStreamServer() : Node("webrtc_stream_server")
  {
    bind_address_ = this->declare_parameter("bind_address", "0.0.0.0");
    port_ = this->declare_parameter("port", 8554);
    base_path_ = this->declare_parameter("base_path", std::string("/webrtc"));
    input_topic_ = this->declare_parameter("input_topic", std::string("/image_raw/compressed"));
    fps_ = this->declare_parameter("fps", 30);
    stun_server_ = this->declare_parameter("stun_server", std::string(""));

    if (port_ <= 0 || port_ > 65535) {
      RCLCPP_WARN(this->get_logger(), "Invalid port %d, fallback to 8554", port_);
      port_ = 8554;
    }
    if (base_path_.empty() || base_path_[0] != '/') {
      base_path_ = "/" + base_path_;
    }
    if (fps_ <= 0) {
      fps_ = 30;
    }

    initGStreamer();

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
        pushJpegToAppSrc(msg->data);
      });

    server_thread_ = std::thread([this]() { this->runHttpServer(); });

    RCLCPP_INFO(
      this->get_logger(),
      "WebRTC server started at http://%s:%d%s (topic: %s)",
      bind_address_.c_str(), port_, base_path_.c_str(), input_topic_.c_str());
  }

  ~WebrtcStreamServer() override
  {
    running_ = false;

    {
      std::lock_guard<std::mutex> lock(server_mutex_);
      if (listen_fd_ >= 0) {
        ::shutdown(listen_fd_, SHUT_RDWR);
        ::close(listen_fd_);
        listen_fd_ = -1;
      }
    }

    if (server_thread_.joinable()) {
      server_thread_.join();
    }

    shutdownGStreamer();
  }

private:
  static bool hasFactory(const char * name)
  {
    GstElementFactory * f = gst_element_factory_find(name);
    if (f == nullptr) {
      return false;
    }
    gst_object_unref(f);
    return true;
  }

  void verifyRuntimeWebrtcDeps()
  {
    // webrtcbin requires libnice elements at runtime for ICE transport.
    if (!hasFactory("nicesrc") || !hasFactory("nicesink")) {
      throw std::runtime_error(
              "Missing GStreamer libnice runtime elements (nicesrc/nicesink). "
              "Install package: gstreamer1.0-nice");
    }
  }

  void initGStreamer()
  {
    std::lock_guard<std::mutex> lock(gst_mutex_);

    verifyRuntimeWebrtcDeps();

    initGStreamerLocked();
  }

  void initGStreamerLocked()
  {
    // Assumes gst_mutex_ is held.

    pipeline_ = gst_pipeline_new("mvcam-webrtc-pipeline");
    if (pipeline_ == nullptr) {
      throw std::runtime_error("Failed to create GStreamer pipeline");
    }

    appsrc_ = gst_element_factory_make("appsrc", "src");
    GstElement * q1 = gst_element_factory_make("queue", "q1");
    GstElement * jpegdec = gst_element_factory_make("jpegdec", "jpegdec");
    GstElement * conv = gst_element_factory_make("videoconvert", "conv");
    GstElement * q2 = gst_element_factory_make("queue", "q2");
    GstElement * vp8enc = gst_element_factory_make("vp8enc", "vp8enc");
    GstElement * pay = gst_element_factory_make("rtpvp8pay", "pay");
    webrtcbin_ = gst_element_factory_make("webrtcbin", "webrtc");

    if (!appsrc_ || !q1 || !jpegdec || !conv || !q2 || !vp8enc || !pay || !webrtcbin_) {
      throw std::runtime_error("Missing required GStreamer elements (appsrc/jpegdec/vp8enc/rtpvp8pay/webrtcbin)");
    }

    g_object_set(appsrc_, "is-live", TRUE, "do-timestamp", TRUE, "format", GST_FORMAT_TIME, nullptr);

    // JPEG input caps. Width/height can be inferred from JPEG; provide framerate.
    GstCaps * caps = gst_caps_new_simple(
      "image/jpeg",
      "framerate", GST_TYPE_FRACTION, fps_, 1,
      nullptr);
    gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
    gst_caps_unref(caps);

    // Keep latency low-ish.
    g_object_set(q1, "leaky", 2 /* downstream */, "max-size-buffers", 2, nullptr);
    g_object_set(q2, "leaky", 2 /* downstream */, "max-size-buffers", 2, nullptr);

    g_object_set(vp8enc, "deadline", 1, "keyframe-max-dist", 30, nullptr);
    g_object_set(pay, "pt", 96, nullptr);

    if (!stun_server_.empty()) {
      // Example: stun://stun.l.google.com:19302
      g_object_set(webrtcbin_, "stun-server", stun_server_.c_str(), nullptr);
    }

    gst_bin_add_many(GST_BIN(pipeline_), appsrc_, q1, jpegdec, conv, q2, vp8enc, pay, webrtcbin_, nullptr);
    if (!gst_element_link_many(appsrc_, q1, jpegdec, conv, q2, vp8enc, pay, nullptr)) {
      throw std::runtime_error("Failed to link GStreamer elements");
    }

    GstPad * pay_src = gst_element_get_static_pad(pay, "src");
    if (pay_src == nullptr) {
      throw std::runtime_error("Failed to get payloader src pad");
    }

    GstCaps * rtp_caps = makeVp8RtpCaps();
    GstWebRTCRTPTransceiver * transceiver = nullptr;
    g_signal_emit_by_name(
      webrtcbin_, "add-transceiver",
      GST_WEBRTC_RTP_TRANSCEIVER_DIRECTION_SENDONLY,
      rtp_caps,
      &transceiver);

    GstPad * webrtc_sink = gst_element_get_static_pad(webrtcbin_, "sink_0");
    if (webrtc_sink == nullptr) {
      webrtc_sink = requestWebrtcSinkPad(webrtcbin_);
    }

    if (transceiver != nullptr) {
      gst_object_unref(transceiver);
    }
    gst_caps_unref(rtp_caps);

    if (webrtc_sink == nullptr) {
      gst_object_unref(pay_src);
      throw std::runtime_error(
              "Failed to request webrtcbin sink pad (sink_%u). "
              "Check installed GStreamer webrtc plugin compatibility.");
    }

    const GstPadLinkReturn link_res = gst_pad_link(pay_src, webrtc_sink);
    gst_object_unref(pay_src);
    gst_object_unref(webrtc_sink);

    if (link_res != GST_PAD_LINK_OK) {
      throw std::runtime_error("Failed to link payloader to webrtcbin");
    }

    g_signal_connect(webrtcbin_, "on-ice-candidate", G_CALLBACK(&WebrtcStreamServer::onIceCandidateStatic), this);

    GstBus * bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, &WebrtcStreamServer::onBusMessageStatic, this);
    gst_object_unref(bus);

    // Keep pipeline READY until we get an offer.
    gst_element_set_state(pipeline_, GST_STATE_READY);
    session_started_ = false;
  }

  void shutdownGStreamer()
  {
    std::lock_guard<std::mutex> lock(gst_mutex_);

    shutdownGStreamerLocked();
  }

  void shutdownGStreamerLocked()
  {
    // Assumes gst_mutex_ is held.
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      appsrc_ = nullptr;
      webrtcbin_ = nullptr;
    }

    {
      std::lock_guard<std::mutex> c_lock(candidate_mutex_);
      pending_candidates_.clear();
    }

    session_started_ = false;
  }

  static gboolean onBusMessageStatic(GstBus *, GstMessage * msg, gpointer user_data)
  {
    return static_cast<WebrtcStreamServer *>(user_data)->onBusMessage(msg);
  }

  gboolean onBusMessage(GstMessage * msg)
  {
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * dbg = nullptr;
        gst_message_parse_error(msg, &err, &dbg);
        RCLCPP_ERROR(
          this->get_logger(), "GStreamer ERROR: %s (%s)",
          err ? err->message : "unknown", dbg ? dbg : "no debug");
        if (err) {
          g_error_free(err);
        }
        if (dbg) {
          g_free(dbg);
        }
      } break;
      case GST_MESSAGE_WARNING: {
        GError * err = nullptr;
        gchar * dbg = nullptr;
        gst_message_parse_warning(msg, &err, &dbg);
        RCLCPP_WARN(
          this->get_logger(), "GStreamer WARNING: %s (%s)",
          err ? err->message : "unknown", dbg ? dbg : "no debug");
        if (err) {
          g_error_free(err);
        }
        if (dbg) {
          g_free(dbg);
        }
      } break;
      default:
        break;
    }

    return TRUE;
  }

  static void onIceCandidateStatic(GstElement *, guint mline_index, gchar * candidate, gpointer user_data)
  {
    static_cast<WebrtcStreamServer *>(user_data)->onIceCandidate(static_cast<int>(mline_index), candidate);
  }

  void onIceCandidate(int mline_index, const gchar * candidate)
  {
    if (candidate == nullptr) {
      return;
    }

    std::lock_guard<std::mutex> lock(candidate_mutex_);
    pending_candidates_.push_back(IceCandidate{mline_index, std::string(candidate)});
    if (pending_candidates_.size() > 512) {
      pending_candidates_.pop_front();
    }
  }

  void pushJpegToAppSrc(const std::vector<uint8_t> & jpeg)
  {
    std::lock_guard<std::mutex> lock(gst_mutex_);
    if (pipeline_ == nullptr || appsrc_ == nullptr) {
      return;
    }

    // If not negotiated/streaming yet, drop frames.
    GstState state = GST_STATE_NULL;
    gst_element_get_state(pipeline_, &state, nullptr, 0);
    if (state != GST_STATE_PLAYING && state != GST_STATE_PAUSED) {
      return;
    }

    GstBuffer * buf = gst_buffer_new_allocate(nullptr, jpeg.size(), nullptr);
    if (buf == nullptr) {
      return;
    }

    GstMapInfo map;
    if (!gst_buffer_map(buf, &map, GST_MAP_WRITE)) {
      gst_buffer_unref(buf);
      return;
    }
    std::memcpy(map.data, jpeg.data(), jpeg.size());
    gst_buffer_unmap(buf, &map);

    const GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buf);
    if (ret != GST_FLOW_OK) {
      // Buffer ownership is transferred; on error it might still be consumed. Do not unref.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "appsrc push_buffer returned %d", static_cast<int>(ret));
    }
  }

  std::optional<std::string> handleOfferSdp(const std::string & offer_sdp)
  {
    std::lock_guard<std::mutex> lock(gst_mutex_);

    // Recreate pipeline on every offer for clean reconnects.
    shutdownGStreamerLocked();
    initGStreamerLocked();

    if (pipeline_ == nullptr || webrtcbin_ == nullptr) {
      return std::nullopt;
    }

    // Start (or restart) streaming state.
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    auto offer_desc = parseOfferSdp(offer_sdp);
    if (!offer_desc) {
      return std::nullopt;
    }

    // Set remote offer.
    g_signal_emit_by_name(webrtcbin_, "set-remote-description", *offer_desc, nullptr);

    // Create answer synchronously.
    GstPromise * promise = gst_promise_new();
    g_signal_emit_by_name(webrtcbin_, "create-answer", nullptr, promise);
    gst_promise_wait(promise);

    const GstStructure * reply = gst_promise_get_reply(promise);
    GstWebRTCSessionDescription * answer = nullptr;
    if (reply != nullptr) {
      gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
    }

    gst_promise_unref(promise);

    if (answer == nullptr) {
      gst_webrtc_session_description_free(*offer_desc);
      return std::nullopt;
    }

    g_signal_emit_by_name(webrtcbin_, "set-local-description", answer, nullptr);

    const std::string answer_sdp = sdpMessageToText(answer->sdp);

    gst_webrtc_session_description_free(answer);
    gst_webrtc_session_description_free(*offer_desc);

    session_started_ = true;

    return answer_sdp;
  }

  void addRemoteIceCandidate(int mline_index, const std::string & candidate)
  {
    std::lock_guard<std::mutex> lock(gst_mutex_);
    if (webrtcbin_ == nullptr) {
      return;
    }
    g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", static_cast<guint>(mline_index), candidate.c_str());
  }

  std::string drainLocalIceCandidatesText()
  {
    std::deque<IceCandidate> drained;
    {
      std::lock_guard<std::mutex> lock(candidate_mutex_);
      drained.swap(pending_candidates_);
    }

    std::ostringstream oss;
    for (const auto & c : drained) {
      oss << c.mline_index << '|' << c.candidate << '\n';
    }
    return oss.str();
  }

  void runHttpServer()
  {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create HTTP socket");
      return;
    }

    int reuse = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, bind_address_.c_str(), &addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid bind address: %s", bind_address_.c_str());
      ::close(fd);
      return;
    }

    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed on %s:%d", bind_address_.c_str(), port_);
      ::close(fd);
      return;
    }

    if (::listen(fd, 8) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Listen failed");
      ::close(fd);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(server_mutex_);
      listen_fd_ = fd;
    }

    while (running_ && rclcpp::ok()) {
      sockaddr_in client_addr;
      socklen_t client_len = sizeof(client_addr);
      const int client_fd = ::accept(fd, reinterpret_cast<sockaddr *>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (!running_ || !rclcpp::ok()) {
          break;
        }
        continue;
      }

      handleHttpClient(client_fd);
      ::close(client_fd);
    }
  }

  void handleHttpClient(int client_fd)
  {
    HttpRequest req;
    if (!parseHttpRequest(client_fd, &req)) {
      return;
    }

    const std::string offer_path = base_path_ + "/offer";
    const std::string cand_post_path = base_path_ + "/candidate";
    const std::string cand_get_path = base_path_ + "/candidates";

    if (req.method == "GET" && (req.path == base_path_ || req.path == "/")) {
      const std::string body = indexHtml(base_path_);
      sendAll(client_fd, httpResponse(200, "OK", "text/html; charset=utf-8", body));
      return;
    }

    if (req.method == "POST" && req.path == offer_path) {
      const auto answer = handleOfferSdp(req.body);
      if (!answer) {
        sendAll(client_fd, httpResponse(400, "Bad Request", "text/plain; charset=utf-8", "Invalid offer"));
        return;
      }
      sendAll(client_fd, httpResponse(200, "OK", "application/sdp", *answer));
      return;
    }

    if (req.method == "POST" && req.path == cand_post_path) {
      const auto mline_str = queryParam(req.query, "mline");
      if (!mline_str) {
        sendAll(client_fd, httpResponse(400, "Bad Request", "text/plain; charset=utf-8", "Missing mline"));
        return;
      }
      int mline = 0;
      try {
        mline = std::stoi(*mline_str);
      } catch (...) {
        sendAll(client_fd, httpResponse(400, "Bad Request", "text/plain; charset=utf-8", "Invalid mline"));
        return;
      }

      if (!req.body.empty()) {
        addRemoteIceCandidate(mline, req.body);
      }

      sendAll(client_fd, httpNoContent(204, "No Content"));
      return;
    }

    if (req.method == "GET" && req.path == cand_get_path) {
      const std::string body = drainLocalIceCandidatesText();
      sendAll(client_fd, httpResponse(200, "OK", "text/plain; charset=utf-8", body));
      return;
    }

    // Fallback: simple 404.
    sendAll(client_fd, httpResponse(404, "Not Found", "text/plain; charset=utf-8", "Not found"));
  }

  std::string bind_address_;
  int port_;
  std::string base_path_;
  std::string input_topic_;
  int fps_;
  std::string stun_server_;

  std::atomic<bool> running_{true};
  std::thread server_thread_;
  std::mutex server_mutex_;
  int listen_fd_{-1};

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  std::mutex gst_mutex_;
  GstElement * pipeline_{nullptr};
  GstElement * appsrc_{nullptr};
  GstElement * webrtcbin_{nullptr};
  bool session_started_{false};

  std::mutex candidate_mutex_;
  std::deque<IceCandidate> pending_candidates_;
};

}  // namespace mvcam_streamer

int main(int argc, char ** argv)
{
  gst_init(nullptr, nullptr);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<mvcam_streamer::WebrtcStreamServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
