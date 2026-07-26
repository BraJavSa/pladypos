# PlaDyPos: ROS 2 Control, Actuator Allocation & Telemetry Package

ROS 2 package for control, actuator mixing, sensor telemetry, and camera streaming of the **PlaDyPos USV**, developed for **ROS 2 Jazzy**.

---

## 1. System Requirements & Prerequisites

This package is designed and tested on **Ubuntu 24.04** running **ROS 2 Jazzy**.

### 1.1 Install ROS 2 Development Tools
Ensure you have the standard developer tools and ROS 2 dependencies:
```bash
sudo apt update
sudo apt install -y ros-dev-tools
```

### 1.2 Install Python Dependencies
The camera streaming and motor mapping nodes require OpenCV and YAML. Run this on both the GCS and the USV:
```bash
sudo apt install -y python3-opencv python3-yaml python3-numpy
```

---

## 2. Arduino CLI Terminal Setup

The onboard Arduino Micro manages the DC-DC regulator, thruster PWM signals, watchdog safety loop, and motor telemetry feedback. Follow these steps to configure and upload code using only the terminal.

### 2.1 Install Arduino CLI
Run the following script to install the latest version of `arduino-cli` into your local path:
```bash
# Download and install the binary
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Move it to a folder in your PATH (e.g., /usr/local/bin)
sudo mv bin/arduino-cli /usr/local/bin/

# Verify the installation
arduino-cli version
```

### 2.2 Configure Board Indexes & Cores
To target the Arduino Micro, download and install the AVR core:
```bash
# Update board index
arduino-cli core update-index

# Install AVR board package (for Arduino Micro / Leonardo / Uno / Mega)
arduino-cli core install arduino:avr
```

### 2.3 Compile and Upload Firmware
From the root of your workspace:
```bash
# Compile the sketch
arduino-cli compile --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/

# Upload the sketch (Make sure to verify the port name, e.g. /dev/ttyACM0)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/
```

---

## 3. Compiling the ROS 2 Workspace

Once dependencies are installed, compile the package in your workspace folder (`~/ros2_ws` or `~/ros2usv_ws`):
```bash
# Compile the pladypos package
colcon build --packages-select pladypos --symlink-install

# Source the workspace setup script
source install/setup.bash
```

---

## 4. Launching the System

The control pipeline is split between the GCS (Ground Control Station) and the USV (Barco) for performance and safety.

### 4.1 On the USV (Barco)
Launches the hardware drivers (IMU, Serial bridge to Arduino Micro) and the camera driver:
```bash
ros2 launch pladypos usv_core.launch.py
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

### 5.2 Real-time Kinect Camera Streaming
The camera driver publishes a highly compressed JPEG stream over Wi-Fi to preserve network bandwidth. To view the stream on your GCS:
```bash
ros2 run rqt_image_view rqt_image_view
```
Select the topic `/usv5/camera/image_raw/compressed` from the dropdown menu to view the low-latency real-time video feed.

---

## 6. Vehicle Characteristics (Physical Platform)

* **Dimensions:** 756 x 756 x 280 mm
* **Weight:** 25.0 kg
* **Payload:** 5.0 kg + water displacement
* **Maneuverable:** 4 thrusters in an X-configuration enable omnidirectional motion and dynamic positioning (DP) at arbitrary orientations.
* **Development Context:** The physical platform was designed by LABUST (see [PlaDyFleet Website](https://pladyfleet.fer.hr/pladyfleet/pladypos)). This ROS 2 package update was developed by the German Research Center for Artificial Intelligence (DFKI).