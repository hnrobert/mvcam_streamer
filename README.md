# mvcam_streamer

ROS2 package for Hikvision industrial camera with compressed image publishing and WebRTC streaming.

Repository: <https://github.com/hnrobert/mvcam_streamer>

## Features

- Publish raw image topic: `/image_raw`
- Publish JPEG compressed topic: `/image_raw/compressed`
- WebRTC real-time streaming: `http://0.0.0.0:8554/webrtc`

## Build

```bash
colcon build --packages-select mvcam_streamer
source install/setup.bash
```

## Run

```bash
ros2 launch mvcam_streamer mvcam_streamer.launch.py
```

## Camera Parameters

- `exposure_time` - 曝光时间 (default: 6000.0)
- `gain` - 增益 (default: 15.0)
- `rotation_angle` - 旋转角度: 0=None, 1=90deg, 2=180deg, 3=270deg (default: 2)
- `publish_compressed` - 是否发布压缩图像 (default: true)
- `jpeg_quality` - JPEG质量 1-100 (default: 60)

## WebRTC Stream Parameters

- `start_webrtc` - 启动WebRTC服务器 (default: true)
- `webrtc_bind_address` - 绑定地址 (default: 0.0.0.0)
- `webrtc_port` - 端口 (default: 8554)
- `webrtc_base_path` - 基础路径 (default: /webrtc)
- `input_topic` - 输入话题 (default: /image_raw/compressed)
- `fps` - 帧率 (default: 30)
- `stun_server` - STUN服务器 (可选, 例如: stun://stun.l.google.com:19302)

## 依赖项

WebRTC流需要安装GStreamer及相关插件：

```bash
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-nice libgstreamer1.0-dev
```
