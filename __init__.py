"""
This package provides methods to initialize, start grabbing, 
stop grabbing, and close the HikVision camera device 
using ROS2 nodes instead of directly using the MvCameraControl SDK.
"""

import os
from typing import Optional

from aim_logger import MvCamLogger
from aim_ros_manager import AimRosLauncher


class AimMVCamManager:
    def __init__(self):
        self.node = None
        self.camera_launcher: Optional[AimRosLauncher] = None
        self.g_bExit = False
        self.logger = MvCamLogger
        self.node_name = "camera_subscriber"

    def start_grabbing(self):
        """
        Start the HikVision camera node and begin grabbing frames
        """
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            ros2_hik_dir = os.path.join(current_dir, "ros2_hik_camera")

            # Create the ROS launcher for the camera
            self.camera_launcher = AimRosLauncher(
                "hik_camera", logger=self.logger)

            # Set working directory and setup commands
            self.camera_launcher.set_working_directory(ros2_hik_dir)
            self.camera_launcher.add_setup_command(
                "source /opt/ros/humble/setup.bash")
            self.camera_launcher.add_setup_command(
                "source ./install/setup.bash")

            # Launch the camera node
            if self.camera_launcher.launch("hik_camera.launch.py"):
                self.logger.info(
                    f"Successfully started HikVision camera node in {ros2_hik_dir}")
                return True
            else:
                self.logger.error("Failed to start HikVision camera node")
                return False

        except Exception as e:
            self.logger.error(f"Failed to start grabbing: {str(e)}")
            return False

    def stop_grabbing(self):
        """Stop the HikVision camera node and grabbing frames"""
        self.g_bExit = True

        if self.camera_launcher:
            success = self.camera_launcher.stop()
            if success:
                self.logger.info("Successfully stopped HikVision camera node")
            self.camera_launcher = None
            return success
        return False
