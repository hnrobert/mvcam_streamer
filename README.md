# mvcam_streamer

ROS2 package for Hikvision industrial camera with compressed image publishing and local video streaming.

Repository: https://github.com/hnrobert/mvcam_streamer

## Features

- Publish raw image topic: `/image_raw`
- Publish JPEG compressed topic: `/image_raw/compressed`
- Stream MJPEG over local HTTP server: `http://127.0.0.1:8080/stream.mjpg`

## Build

```bash
colcon build --packages-select mvcam_streamer
source install/setup.bash
```

## Run

```bash
ros2 launch mvcam_streamer mvcam_streamer.launch.py
```

## Main Parameters

- `exposure_time`
- `gain`
- `rotation_angle` (0, 1, 2, 3)
- `publish_compressed` (true/false)
- `jpeg_quality` (1-100)

## Stream Parameters

- `start_stream_server` (true/false)
- `stream_bind_address` (default: 127.0.0.1)
- `stream_port` (default: 8080)
- `stream_path` (default: /stream.mjpg)
