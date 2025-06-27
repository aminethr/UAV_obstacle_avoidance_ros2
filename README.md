# Obstacle Avoidance ROS 2 Node (Direct Method with Depth Image)

This ROS 2 node performs obstacle avoidance using a depth camera feed and an XGBoost regression model to estimate the safe stopping distance of a drone. The model predicts how much space the drone needs to slow down and begin avoiding obstacles based on its current speed. This ensures that at higher speeds, the drone begins obstacle checking and maneuvering earlier to compensate for longer stopping distances.

When an obstacle is too close, the depth image is divided into 16 equally-sized regions (4 rows × 4 columns). The node analyzes each region and selects the one with the highest average depth — representing the most open direction — and steers the drone toward it.

## 📌 Features

- Subscribes to depth images from a camera sensor.
- Uses an XGBoost model to estimate stopping distance based on current velocity.
- Starts obstacle checks earlier at higher speeds.
- Dynamically adjusts drone velocity and direction for safe navigation.
- Divides the depth image into a 4×4 grid (16 regions) to detect the most open direction.
- Selects and aligns with the region that has the largest mean depth value.
- Publishes visual debug output showing the selected region of interest.
- Direct method using only real-time depth and velocity (no mapping).

## 📷 Topics

### Subscribed

- `/drone0_camera1_sensor/depth/image_raw` — Depth image input (`sensor_msgs/Image`)
- `/drone0/local_position/velocity_local` — Drone velocity (`geometry_msgs/TwistStamped`)

> 📝 **Note**: These topic names depend on how you defined the topics in your drone's SDF (or URDF) model. Make sure they match your simulation setup.

### Published

- `/drone0/setpoint_velocity/cmd_vel_unstamped` — Drone movement command (`geometry_msgs/Twist`)
- `/debug/depth_regions` — Debug image showing selected depth region (`sensor_msgs/Image`)

## 📦 Dependencies

Make sure you have the following installed in your ROS 2 Python environment:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`
- `opencv-python`
- `numpy`
- `joblib` (for loading the machine learning model)

To install Python dependencies:
```bash
pip install opencv-python numpy joblib.
```
For cv_bridge, install via ROS:
```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge
```
## 🧠 Model File

This node loads an XGBoost model that estimates the stopping distance required at a given speed. This allows the drone to determine how early it should begin checking for obstacles — especially useful at higher speeds when more distance is needed to slow down or maneuver.

📌 **Note**:  
- The model was trained on drone velocities ranging from **0 to 10 m/s**.
- It was developed and tested in **Gazebo** simulation under specific environmental and sensor conditions.

If your drone operates at significantly different speeds or in very different environments (e.g., real-world wind or lighting), you may want to retrain the model with data from your own setup for better accuracy.


Make sure the model is saved at the expected path:

xgboost_stopping_distance_model.joblib

If your path is different, change this line in the code:

self.model = joblib.load("path/to/your/model.joblib")

## 🚀 Running the Node

    Source your ROS 2 workspace:

source /opt/ros/<your-distro>/setup.bash
source ~/your_ws/install/setup.bash

    Run the node:

ros2 run <your_package_name> obstacle_avoidance_direct_node

Replace <your_package_name> with the name of your ROS 2 package.
🛠 Node Parameters (Optional Tuning)

    K: Proportional gain for horizontal/vertical alignment

    current_speed: The Speed of which the drone will move wtih

    twist.linear.y: The Speed of which the drone will move wtih when its avoiding the obstacle

    max_x_speed, max_z_speed: Limit the lateral and vertical speeds

    safety_margin: Extra buffer added to the predicted stopping distance

    deadzone: Threshold below which no alignment correction is applied


## 🧪 Example Debug Output

The node publishes a debug image to /debug/depth_regions. You can visualize it using:

rqt_image_view /debug/depth_regions

### 🚁 Simple Example

Below is a basic demonstration of the obstacle avoidance system in action.

📍 **Simulation Environment**: This example is tested in the **Gazebo 3D simulator** using **ArduPilot** with the **ArduCopter** firmware.

![Simple Example](https://github.com/user-attachments/assets/ec9d1004-d95a-4b22-b5bb-ba14ac0d94b3)
