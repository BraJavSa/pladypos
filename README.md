# PlaDyPos: ROS 2 Control, Actuator Allocation & Telemetry Package

ROS 2 package for control, actuator mixing, sensor telemetry, and camera/depth streaming of the **PlaDyPos USV**, developed for **ROS 2 Jazzy**.

---

## 1. System Requirements & Prerequisites

This package is designed and tested on **Ubuntu 24.04** running **ROS 2 Jazzy**.

### 1.1 Install ROS 2 Development Tools
Ensure you have the standard developer tools and ROS 2 dependencies:
```bash
sudo apt update
sudo apt install -y ros-dev-tools
```

### 1.2 Install Python & ROS 2 Dependencies
The camera streaming and motor mapping nodes require OpenCV, cv_bridge, and YAML. Run this on both the GCS and the USV:
```bash
sudo apt install -y python3-opencv python3-yaml python3-numpy ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager ros-jazzy-depth-image-proc
```

---

## 2. Xbox 360 Kinect (Kinect v1) Installation on USV

For full RGB-D camera and 3D PointCloud capabilities on the USV, the community-maintained driver `kinect_ros2` is used.

### 2.1 Build and Install `libfreenect` from Source
Due to the lack of CMake configuration files in the standard `apt` package for `libfreenect` on Ubuntu 24.04, it must be compiled from source on the USV:
```bash
# Install compilation and USB dependencies
sudo apt update
sudo apt install -y git cmake pkg-config build-essential libusb-1.0-0-dev freeglut3-dev

# Clone and compile libfreenect
cd ~
git clone https://github.com/OpenKinect/libfreenect.git
cd libfreenect
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=ON
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 2.2 Clone the `kinect_ros2` Package
Clone the repository into the USV workspace (`~/ros2usv_ws`):
```bash
cd ~/ros2usv_ws/src
git clone https://github.com/fadlio/kinect_ros2.git
```

### 2.3 Patch for ROS 2 Jazzy compatibility
In ROS 2 Jazzy, `cv_bridge.h` is removed in favor of `cv_bridge.hpp`. Run the following `sed` patch on the USV before compiling:
```bash
find ~/ros2usv_ws/src/kinect_ros2 -type f -exec sed -i 's/cv_bridge\/cv_bridge.h/cv_bridge\/cv_bridge.hpp/g' {} +
```

### 2.4 Compile the Workspace on the USV
```bash
cd ~/ros2usv_ws
rm -rf build/ install/ log/  # Clear cache
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 3. Arduino CLI Terminal Setup

The onboard Arduino Micro manages the DC-DC regulator, thruster PWM signals, watchdog safety loop, and motor telemetry feedback. Follow these steps to configure and upload code using only the terminal.

### 3.1 Install Arduino CLI
Run the following script to install the latest version of `arduino-cli` into your local path:
```bash
# Download and install the binary
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Move it to a folder in your PATH (e.g., /usr/local/bin)
sudo mv bin/arduino-cli /usr/local/bin/

# Verify the installation
arduino-cli version
```

### 3.2 Configure Board Indexes & Cores
To target the Arduino Micro, download and install the AVR core:
```bash
# Update board index
arduino-cli core update-index

# Install AVR board package (for Arduino Micro / Leonardo / Uno / Mega)
arduino-cli core install arduino:avr
```

### 3.3 Compile and Upload Firmware
From the root of your workspace:
```bash
# Compile the sketch
arduino-cli compile --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/

# Upload the sketch (Make sure to verify the port name, e.g. /dev/ttyACM0)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/
```

---

## 4. Launching the System

The control pipeline is split between the GCS (Ground Control Station) and the USV (Barco) for performance and safety.

### 4.1 On the USV (Barco)
1. **Launch Core Drivers (IMU, Arduino Bridge):**
   ```bash
   ros2 launch pladypos usv_core.launch.py
   ```
2. **Launch Kinect Driver (RGB + Depth + 3D PointCloud):**
   In a separate terminal on the USV:
   ```bash
   ros2 launch kinect_ros2 pointcloud.launch.py
   ```

### 4.2 On the GCS (PC with Joystick)
Launches the physical joystick node:
```bash
ros2 launch pladypos gcs.launch.py
```

---

## 5. Diagnostic & Utility Tools

### 5.1 Motor Mapping Calibration GUI
If thruster wires are swapped or physical configurations change, run this visual wizard on your GCS to identify which channel maps to which physical motor corner:
```bash
ros2 run pladypos motor_mapper.py
```
This utility will pulse each motor at 30% thrust for 4 seconds, ask you to select its physical location, and save the result dynamically to `config/motor_mapping.yaml`. The mixer node (`teleop_mixer.py`) automatically loads this file at startup.

### 5.2 Real-time Kinect Camera & PointCloud Visualization
* **2D Video Feed:** To view RGB/Depth feeds, open a terminal on your GCS and run:
  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```
  Select the topic `/kinect/rgb/image_raw/compressed` or `/kinect/depth/image_raw` from the dropdown menu.
* **3D PointCloud:** To view the 3D depth point cloud, open a terminal on your GCS and run:
  ```bash
  rviz2
  ```
  Add a `PointCloud2` display and subscribe to `/kinect/depth/color/points`.

---

## 6. Vehicle Characteristics (Physical Platform)

* **Dimensions:** 756 x 756 x 280 mm
* **Weight:** 25.0 kg
* **Payload:** 5.0 kg + water displacement
* **Maneuverable:** 4 thrusters in an X-configuration enable omnidirectional motion and dynamic positioning (DP) at arbitrary orientations.
* **Development Context:** The physical platform was designed by LABUST (see [PlaDyFleet Website](https://pladyfleet.fer.hr/pladyfleet/pladypos)). This ROS 2 package update was developed by the German Research Center for Artificial Intelligence (DFKI).