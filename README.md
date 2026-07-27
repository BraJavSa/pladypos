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
The camera streaming, point cloud, and motor mapping nodes require OpenCV, cv_bridge, Boost, and YAML. Run this on both the GCS and the USV:
```bash
sudo apt update
sudo apt install -y python3-opencv python3-yaml python3-numpy libboost-all-dev \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-camera-info-manager \
  ros-jazzy-depth-image-proc ros-jazzy-compressed-depth-image-transport \
  ros-jazzy-compressed-image-transport
```

---

## 2. Xbox One Kinect (Kinect v2) Installation on USV

For full RGB-D camera and 3D PointCloud capabilities on the USV, the community-maintained driver `kinect2_ros2` is used.

### 2.1 Build and Install `libfreenect2` from Source
Due to the lack of prepackaged binaries for `libfreenect2` on Ubuntu 24.04, it must be compiled from source on the USV:
```bash
# 1. Install compilation and USB dependencies
sudo apt update
sudo apt install -y build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libtool autoconf libudev-dev

# 2. Clone and compile libfreenect2
cd ~
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)
sudo make install
```

### 2.2 Configure USB Permissions & Blacklist Conflicting Kernel Modules
The default Linux kernel driver `gspca_kinect` conflicts with `libfreenect2` by claiming the USB device. We must blacklist it and set up permissions on the USV:
```bash
# 1. Install the Kinect v2 udev rules
sudo cp ~/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# 2. Unload currently loaded conflicting kernel modules
sudo modprobe -r gspca_kinect gspca_main

# 3. Permanently blacklist conflicting modules
echo "blacklist gspca_kinect" | sudo tee -a /etc/modprobe.d/blacklist.conf
echo "blacklist gspca_main" | sudo tee -a /etc/modprobe.d/blacklist.conf
```
*Note: Unplug and replug the Kinect v2's USB 3.0 connector to reload permissions.*

### 2.3 Clone and Patch `kinect2_ros2`
Clone the driver into the USV workspace (`~/ros2usv_ws`) and apply compatibility patches for ROS 2 Jazzy:
```bash
# 1. Clone the driver
cd ~/ros2usv_ws/src
git clone https://github.com/YuLiHN/kinect2_ros2.git

# 2. Patch: Replace deprecated cv_bridge.h with cv_bridge.hpp
find ~/ros2usv_ws/src/kinect2_ros2 -type f -exec sed -i 's/cv_bridge\/cv_bridge.h/cv_bridge\/cv_bridge.hpp/g' {} +

# 3. Patch: Set 'sensor' serial parameter to an empty string '' in kinect2_bridge.launch.py.
# This prevents ROS 2 YAML type parsing issues (interpreting numeric serials with '8' or '9' as double/floats)
# and enables automatic plug-and-play detection of the first connected camera.
sed -i "s/'sensor': '004436460547'/'sensor': ''/g" ~/ros2usv_ws/src/kinect2_ros2/kinect2_bridge/launch/kinect2_bridge.launch.py
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
Choose one of the following unified launcher options:

* **Option A: Full RGB-D & 3D PointCloud Mode** (Vehicle controls + full 3D point cloud streaming):
  ```bash
  ros2 launch pladypos usv_kinect.launch.py
  ```
* **Option B: Webcam-Only Mode** (Vehicle controls + 2D color image streaming only; disables heavy depth/pointcloud processing to save CPU and Wi-Fi bandwidth):
  ```bash
  ros2 launch pladypos usv_webcam.launch.py
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
  Select the topic `/kinect2/qhd/image_color_rect` or `/kinect2/qhd/image_depth_rect`.
* **3D PointCloud:** To view the 3D depth point cloud, open a terminal on your GCS and run:
  ```bash
  rviz2
  ```
  Add a `PointCloud2` display, subscribe to `/kinect2/qhd/points`, and set the **Fixed Frame** to `kinect2_link`.

---

## 6. Vehicle Characteristics (Physical Platform)

* **Dimensions:** 756 x 756 x 280 mm
* **Weight:** 25.0 kg
* **Payload:** 5.0 kg + water displacement
* **Maneuverable:** 4 thrusters in an X-configuration enable omnidirectional motion and dynamic positioning (DP) at arbitrary orientations.
* **Development Context:** The physical platform was designed by LABUST (see [PlaDyFleet Website](https://pladyfleet.fer.hr/pladyfleet/pladypos)). This ROS 2 package update was developed by the German Research Center for Artificial Intelligence (DFKI).

---

## 7. Telemetry & Control Vector Mappings

The USV outputs its current thruster efforts on the ROS 2 topic `motor_feedback` (type `std_msgs/Float32MultiArray`) at **30 Hz**. The values represent normalized thruster forces ranging from `-1.0` (maximum reverse thrust) to `1.0` (maximum forward thrust).

The vector contains exactly 4 elements mapped in the following order:

| Index | Physical Location | Abbreviation | Associated Arduino Pin / Output | Description |
| :---: | :---: | :---: | :---: | --- |
| **0** | Front Right | `FR` | Pin `A0` (Thruster 1) | Normalized force of the front-right thruster |
| **1** | Front Left | `FL` | Pin `A1` (Thruster 2) | Normalized force of the front-left thruster |
| **2** | Back Right | `BR` | Pin `A2` (Thruster 3) | Normalized force of the back-right thruster |
| **3** | Back Left | `BL` | Pin `A3` (Thruster 4) | Normalized force of the back-left thruster |

### Radio Control (Spektrum DSMX) Channels
When using manual Spektrum RC control (Channel 4 is set to `1705`), the channel mappings decoded by the bridge are:
* **Channel 1 (surge / $u$):** Advance motion. Range `342` (backward) to `1702` (forward). Center is `1023`.
* **Channel 2 (sway / $v$):** Lateral motion. Range `343` (right) to `1702` (left). Center is `1023`.
* **Channel 3 (yaw / $r$):** Clockwise/counter-clockwise rotation. Range `343` (clockwise) to `1702` (counter-clockwise). Center is `1023`.
* **Channel 4:** Control Mode Switch:
  * `1705` (High): **Radio Control Mode** (takes priority).
  * `343` (Low): **PC / ROS 2 Joystick Mode** (default fallback when radio signal is off).