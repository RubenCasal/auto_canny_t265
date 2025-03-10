# Auto Canny T265 - ROS2 Package

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)


## 📌 Overview
This **ROS2 package** integrates the **Intel RealSense T265 Tracking Camera** with **OpenCV** for real-time motion tracking and fisheye image processing. The package provides:
- **C++ Node (`t265_node`)**: Publishes **odometry, IMU, and fisheye images**.
- **Python Node (`auto_canny_node`)**: Applies **Auto-Canny edge detection** on fisheye images.
- **Launch File**: Starts both nodes in ROS2.

![Auto-Canny Edge Detection](auto_canny_detection.gif)


## 📸 Features
✔ **Real-time Odometry & IMU Data**  
✔ **Fisheye Image Publishing**  
✔ **TF Broadcast for Localization**  
✔ **Adaptive Canny Edge Detection on Fisheye Images**  
✔ **Seamless ROS2 Integration (Humble)**  

---

## 🚀 Installation & Setup

### Clone the Repository
Ensure you have the Librealsense2 library compatible with your system

```bash
cd ~/ros2_ws/src
git clone git@github.com:RubenCasal/auto_canny_t265.git
cd ~/ros2_ws
colcon build --packages-select auto_canny_t265
source install/setup.bash
```
## 🏁 Running the Package
Run Nodes Separately
### Start the T265 Node (C++)
```
ros2 run auto_canny_t265 t265_node
```
### Start the Auto-Canny Node (Python)
```
ros2 run auto_canny_t265 auto_canny_node
```
### Run with Launch File

To start both nodes together:
```
ros2 launch auto_canny_t265 canny_detection_launch.py
```

## **📡 Published Topics**

This package publishes multiple topics for **navigation and image processing**.

| **Topic Name**                  | **Message Type**                | **Description**                          |
|----------------------------------|--------------------------------|------------------------------------------|
| `/rs_t265/odom`                 | `nav_msgs/msg/Odometry`        | Odometry data (position & pose).        |
| `/rs_t265/imu`                  | `sensor_msgs/msg/Imu`          | IMU data (gyro & acceleration).         |
| `/rs_t265/fisheye_left`         | `sensor_msgs/msg/Image`        | Left fisheye image from the T265.       |
| `/rs_t265/fisheye_right`        | `sensor_msgs/msg/Image`        | Right fisheye image from the T265.      |
| `/rs_t265/canny_edge_detection` | `sensor_msgs/msg/Image`        | Processed edge-detected fisheye image.  |
| `/tf`                           | `tf2_msgs/msg/TFMessage`       | TF transformations for localization.    |


## 🎥 Visualizing Results in RViz2
### 1️⃣ Launch RViz2

Start RViz2:
```
rviz2
```
### 2️⃣ Add Required Displays

Once RViz2 is open:

      Click "Add" → "By Topic".
  
      Select:
          /rs_t265/odom → Odometry (for trajectory visualization).
          /rs_t265/imu → IMU (for orientation data).
          /rs_t265/fisheye_left → Image (for raw camera feed).
          /rs_t265/canny_edge_detection → Image (for edge-detected images).
          /tf → TF (for viewing the transform frames).
  
      Adjust settings as needed and view real-time sensor data.

## Python Node: auto_canny_node.py

The Auto-Canny Node enhances edge detection by dynamically adjusting thresholds based on image characteristics. It processes fisheye images from the Intel RealSense T265, making edge detection more adaptive and robust than the traditional Canny method.
## 🛠 How Auto-Canny Works
### 1️⃣ Bilateral Filtering: Noise Reduction Without Losing Edges

Unlike Gaussian Blur, Bilateral Filtering smooths the image while preserving edges, reducing noise without blurring important details.
### 2️⃣ Adaptive Canny Edge Detection: Smart Threshold Selection

Traditional Canny requires fixed thresholds, making it unreliable under different lighting. Auto-Canny solves this by:

    Computing the median intensity of the image.
    Defining adaptive lower and upper thresholds based on the median.
    Applying Canny edge detection dynamically.

## 🎯 Why Auto-Canny?

✔ **No manual tuning** – Adapts automatically to different images.  
✔ **Better edge detection** – Works well in **low-contrast conditions**.  
✔ **More reliable in real-world applications** – Handles **lighting variations** effectively.  

This makes **Auto-Canny ideal for robotics, SLAM, and vision-based tasks**, where **edge clarity matters** despite changing environments. 🚀  

