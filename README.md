# Edge Detection Task

## Tasks and Results

### 1. Basic
For C++, the `EdgeDetector.hpp` and `EdgeDetector.cpp` files should be implemented. Any required libraries should be added to `CMakeLists.txt`. For Python, `edge_detector.py` should be completed with installation instructions for dependencies.

It should be possible to provide different images as input and display the output with edges detected in green. Additionally, a `README.md` file should include installation instructions, implementation steps, concepts used, and potential improvements.

#### **Result**
Edge detection identifies rapid changes in pixel intensity. The implemented solutions are:
1. **Laplacian Edge Detector:** Identifies edges as zero-crossing points in the second derivative (2D Laplacian) of the image function.
2. **Canny Edge Detector:** Uses a gradient operator (Sobel filter) to compute gradients at each pixel, then applies a 1D Laplacian along the gradient orientation. Strong zero crossings are classified as edges.

**Implementation Details:**
- The image is converted from RGB to grayscale to simplify computation.
- A Gaussian blur is applied to reduce noise, preventing false zero crossings.
- The function `edge_detect` in `edge_detector.py` implements these methods:
  - `0`: Canny Edge Detector (default)
  - `1`: Laplacian Edge Detector
- A commented-out section overlays results from both detectors for comparison.

### 2. Vision_ROS
#### **Task:**
Provide ROS `.srv` and `.msg` files to create a ROS service for edge detection. Implement a client that detects edges in images from a directory.

#### **Result:**
- A ROS service named `Image_edge_detect_service` accepts `input_img_path` (string) and returns `output_img_path` (string), storing the edge-detected image.
- To use:
  - Run `../src/edge_detect_img_node.py` to start the service.

**Additional Task:** Detect edges in images from a ROS topic when playing a `.bag` file, then visualize input and output images in RViz.

**Result:**
- A ROS node `image_raw_listener` subscribes to `/camera/color/image_raw` to receive `sensor_msgs/Image`, processes edge detection, and republishes results to `edge_detected_image`.
- **To perform this:**
  1. Play a provided `.bag` file: `rosbag play --clock -l <path_to_bagfile>`
  2. Run `../src/edge_detect_bag_data_node.py`
  3. Add RViz displays for `/camera/color/image_raw` and `edge_detected_image`

**Additional Task:** Convert detected edge pixels from (u, v) to 3D (x, y, z) using depth images and camera parameters. Publish 3D data to a ROS topic (`edge_points`) of type `sensor_msgs/PointCloud`.

**Result:**
- Using intrinsic camera parameters (`/camera/color/camera_info`) and depth data (`/camera/depth/image_rect_raw`), the 3D coordinates are computed as:
  - `x = depth * (u - cx) / fx`
  - `y = depth * (v - cy) / fy`
  - `z = depth`
- The `edge_points` topic publishes `sensor_msgs/PointCloud`.
- **To perform this:**
  1. Play a `.bag` file: `rosbag play --clock -l <path_to_bagfile>`
  2. Run `../src/edge_detect_bag_data_node.py`
  3. Start publishing `edge_points`: `../src/edge_pointcloud_pub_node.py`
  4. Verify output: `rostopic echo /edge_points`

### 3. Robot_ROS
#### **Task:**
Visualize 3D edge points in RViz as markers alongside a robot model.

#### **Result:**
- Markers are published in `camera_color_optical_frame` since `/camera/...` topics use this frame.
- The `edge_3D_marker` topic publishes `visualization_msgs/Marker`.
- **To view markers:**
  1. Play a `.bag` file: `rosbag play --clock -l <path_to_bagfile>`
  2. Run `../src/edge_detect_bag_data_node.py`
  3. Start point cloud publishing: `../src/edge_pointcloud_pub_node.py`
  4. Open RViz and add a `Marker` display for `/edge_3D_marker`

**Transforming 3D Data to Robot Frames:**
- `TransformListener.transformPoint()` converts 3D coordinates from `camera_color_optical_frame` to robot frames using transformations from `/tf` and `/tf_static`.
- The transformation logic is implemented but commented out in `../src/edge_pointcloud_pub_node.py` due to testing limitations.

### **Room for Improvement:**
- Refactoring Python scripts into classes for better modularity.
- Translating `rospy` implementations into `roscpp` for efficiency.
- Enhancing noise filtering to improve edge detection accuracy.

---
This repository provides robust edge detection solutions with ROS integration, visualization in RViz, and 3D coordinate transformation for robotic applications.
