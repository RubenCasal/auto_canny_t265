# Auto Canny T265 - ROS2 Package

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

## 📌 Overview
This **ROS2 package** integrates the **Intel RealSense T265 Tracking Camera** with **OpenCV** for real-time motion tracking and fisheye image processing. The package provides:
- **C++ Node (`t265_node`)**: Publishes **odometry, IMU, and fisheye images**.
- **Python Node (`auto_canny_node`)**: Applies **Auto-Canny edge detection** on fisheye images.
- **Launch File**: Starts both nodes in ROS2.

## 📸 Features
✔ **Real-time Odometry & IMU Data**  
✔ **Fisheye Image Publishing**  
✔ **TF Broadcast for Localization**  
✔ **Adaptive Canny Edge Detection on Fisheye Images**  
✔ **Seamless ROS2 Integration (Humble)**  

---

## 🚀 Installation & Setup

### 1️⃣ Clone the Repository
```bash
cd ~/ros2_ws/src
git clone git@github.com:RubenCasal/auto_canny_t265.git
cd ~/ros2_ws
colcon build --packages-select auto_canny_t265
source install/setup.bash
2️⃣ Check Dependencies

Ensure you have Intel RealSense SDK and required ROS2 dependencies installed:
sudo apt update
sudo apt install ros-humble-vision-opencv ros-humble-cv-bridge ros-humble-tf2-ros \
                 ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
                 ros-humble-nav-msgs librealsense2 librealsense2-dev

🏁 Running the Package
Start the Nodes

ros2 launch auto_canny_t265 canny_detection_launch.py

This will:

    Start the T265 node (t265_node - C++).
    Start the Auto-Canny node (auto_canny_node - Python).

Check ROS2 Topics

List active topics:

ros2 topic list

Expected topics:

/rs_t265/odom
/rs_t265/imu
/rs_t265/fisheye_left
/rs_t265/fisheye_right
/rs_t265/canny_edge_detection
/tf

Visualize Data
🛰️ Odometry & IMU

ros2 topic echo /rs_t265/odom
ros2 topic echo /rs_t265/imu

📷 Fisheye & Canny Edge Detection

ros2 run rqt_image_view rqt_image_view

Select:

    /rs_t265/fisheye_left
    /rs_t265/fisheye_right
    /rs_t265/canny_edge_detection

⚙️ Nodes Overview
🟢 C++ Node: t265_node

    t265_node handles:

    Intel RealSense T265 Sensor Integration
    Publishing Odometry & IMU Data
    Publishing Left & Right Fisheye Images
    TF Transform Broadcasts

📌 Published Topics:
Topic	Type	Description
/rs_t265/odom	nav_msgs/msg/Odometry	Odometry data (position & pose)
/rs_t265/imu	sensor_msgs/msg/Imu	IMU data (gyro & acceleration)
/rs_t265/fisheye_left	sensor_msgs/msg/Image	Left fisheye image
/rs_t265/fisheye_right	sensor_msgs/msg/Image	Right fisheye image
/tf	tf2_msgs/msg/TFMessage	TF transformation broadcast

📌 TF Frames:

odom → t265_frame

🟡 Python Node: auto_canny_node

    auto_canny_node applies:

    Bilateral Filtering for noise reduction
    Adaptive Canny Edge Detection for feature extraction

📌 Subscribed Topic:
Topic	Type	Description
/rs_t265/fisheye_left	sensor_msgs/msg/Image	Left fisheye image input

📌 Published Topic:
Topic	Type	Description
/rs_t265/canny_edge_detection	sensor_msgs/msg/Image	Processed edge-detected image
📜 Launch File (canny_detection_launch.py)

Launch both nodes:

import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='auto_canny_t265',
            executable='t265_node',
            name='t265_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='auto_canny_t265',
            executable='auto_canny_node',
            name='auto_canny_node',
            output='screen'
        ),
    ])

🎯 Future Improvements

    ✅ Improve pose filtering (Kalman Filter)
    ✅ Add visualization tools (RViz2 integration)
    ✅ Optimize edge detection parameters dynamically
    ✅ Publish processed depth images (if using other RealSense models)

💡 Contribution

Feel free to contribute by submitting:

    Bug fixes
    Feature enhancements
    Documentation updates

Clone & create a feature branch:

git checkout -b feature-branch
git commit -m "New feature"
git push origin feature-branch
