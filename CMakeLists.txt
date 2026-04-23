cmake_minimum_required(VERSION 3.8)
project(mvcam_streamer)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings any more,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mvcam_streamer_node.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC /opt/MVS/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC /opt/MVS/lib/64)
  install(
    DIRECTORY /opt/MVS/lib/64
    DESTINATION lib
  )
  # elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  #   target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  #   install(
  #     DIRECTORY hikSDK/lib/arm64/
  #     DESTINATION lib
  #   )
else()
  message(FATAL_ERROR "Unsupported host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
  ${OpenCV_LIBS}
)

ament_auto_add_executable(mjpeg_stream_server
  src/mjpeg_stream_server.cpp
)

ament_auto_add_executable(h264_rtsp_stream_server
  src/h264_rtsp_stream_server.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mvcam_streamer::MvcamStreamerNode
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)
