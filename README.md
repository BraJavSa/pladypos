# PlaDyPos: ROS 2 Control, Actuator Allocation & Telemetry Package

ROS 2 package for control, actuator mixing, sensor telemetry, and visual localization of the **PlaDyPos USV**, developed for **ROS 2 Jazzy** on **Ubuntu 24.04**.

---

## 1. System Requirements & Prerequisites

This package is designed and tested on **Ubuntu 24.04** running **ROS 2 Jazzy**.

### 1.1 Install ROS 2 Development Tools
Ensure you have the standard developer tools and ROS 2 dependencies installed:
```bash
sudo apt update
sudo apt install -y ros-dev-tools
```

### 1.2 Install Python & ROS 2 Dependencies
Install the required packages for AprilTag vision, motor mixing, teleoperation, and IMU fusion:
```bash
sudo apt update
sudo apt install -y python3-opencv python3-yaml python3-numpy python3-serial \
  libboost-all-dev ros-jazzy-cv-bridge ros-jazzy-image-transport \
  ros-jazzy-camera-info-manager ros-jazzy-imu-filter-madgwick \
  ros-jazzy-joy ros-jazzy-tf2-ros
```

---

## 2. Microcontroller & IMU Firmware Setup (Ubuntu Server 24.04)

Both the onboard **Arduino Micro** (actuators/safety watchdog) and the **SparkFun 9DoF Razor IMU M0** (attitude and telemetry sensor) are programmed via command-line using `arduino-cli`.

### 2.1 Install Arduino CLI
Download and install the official binary to your system PATH:
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/
arduino-cli version
```

Ensure your user has read/write permissions for USB serial devices:
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in, or run: newgrp dialout
```

### 2.2 Configure Board Indexes & Cores
Configure the package manager to support both AVR (Arduino Micro) and SAMD (SparkFun Razor M0):
```bash
# Add SparkFun package index URL
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json

# Update board indexes
arduino-cli core update-index

# Install AVR core (for Arduino Micro)
arduino-cli core install arduino:avr

# Install SparkFun SAMD core (for SparkFun 9DoF Razor IMU M0)
arduino-cli core install SparkFun:samd
```

### 2.3 Compile and Upload Arduino Micro Firmware (`usv_bridge`)
The Arduino Micro sketch manages DC-DC regulation, thruster PWM signals, motor telemetry, and the watchdog safety loop.

From the root of your workspace:
```bash
# Compile the sketch
arduino-cli compile --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/

# Upload the sketch (verify the port, e.g., /dev/ttyACM0 or /dev/ttyACM1)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:micro src/pladypos/arduino/usv_bridge/
```

### 2.4 Compile and Upload SparkFun 9DoF Razor IMU M0 Firmware
The IMU runs the official open-source SparkFun Razor M0 default firmware. It can be cloned temporarily to compile and flash:

1. **Install required libraries:**
```bash
arduino-cli lib install "SparkFun MPU9250-DMP"
arduino-cli lib install "FlashStorage"
```

2. **Clone the official repository to a temporary directory:**
```bash
git clone https://github.com/sparkfun/9DOF_Razor_IMU.git /tmp/9DOF_Razor_IMU
```

3. **Compile the sketch:**
```bash
arduino-cli compile --fqbn SparkFun:samd:samd21_9dof /tmp/9DOF_Razor_IMU/Firmware/_9DoF_Razor_M0_Firmware/
```

4. **Upload the sketch to the IMU:**
```bash
# Verify the serial port (e.g., /dev/ttyACM0 or /dev/ttyACM1)
arduino-cli upload -p /dev/ttyACM0 --fqbn SparkFun:samd:samd21_9dof /tmp/9DOF_Razor_IMU/Firmware/_9DoF_Razor_M0_Firmware/
```
> *Tip: If the serial port is not detected or upload fails, quickly double-press the physical reset button on the Razor IMU M0 board to force it into bootloader mode (the onboard blue/green LED will pulse slowly).*

5. **Clean up temporary files:**
```bash
rm -rf /tmp/9DOF_Razor_IMU
```

---

## 3. Launching the System

The system pipeline is split between the **USV (Boat)** and the **GCS (Ground Control Station)**:

### 3.1 On the USV (Boat)

**Unified USV Launcher (Manual Control + IMU Telemetry):**
Starts the entire onboard pipeline in a single command:
* **Arduino Bridge:** Decodes Spektrum RC radio channels and sends PWM thruster commands.
* **Teleoperation Mixer:** Maps joystick commands (`joy`) to thruster forces.
* **IMU Driver & Madgwick Filter:** Streams calibrated IMU and magnetometer data, fusing orientation into `imu/data`.

```bash
# Standard launch (auto-detects serial ports for Arduino and IMU):
ros2 launch pladypos core.launch.py

# If the gamepad joystick is connected directly to the USV:
ros2 launch pladypos core.launch.py use_joy:=true
```
*(If you only need to run the IMU without the motor/radio bridge, you can still run `ros2 launch pladypos imu.launch.py`)*

### 3.2 On the GCS (Control PC)

1. **Manual Joystick Teleoperation:**
   Publishes joystick input from a connected gamepad:
   ```bash
   ros2 launch pladypos manual_control.launch.py
   ```

2. **Visual Localization & Pose Fusion:**
   Launches the AprilTag tag estimator and visual-IMU fusion nodes:
   ```bash
   ros2 launch pladypos gcs.launch.py
   ```

---

## 4. Diagnostic & Calibration Tools

### 4.1 Motor Mapping Calibration GUI
If thruster wires are swapped or physical configurations change, run this visual wizard on your GCS to identify which channel maps to which physical motor corner:
```bash
ros2 run pladypos motor_mapper.py
```
This utility will pulse each motor at 30% thrust for 4 seconds, prompt you to select its physical location, and dynamically save the configuration to `config/motor_mapping.yaml`. The mixer node (`teleop_mixer.py`) automatically loads this file at startup.

### 4.2 IMU RViz2 Real-Time Visualization
To visualize the real-time attitude of the IMU in RViz2:
1. Run the IMU launch on the USV:
   ```bash
   ros2 launch pladypos imu.launch.py
   ```
2. In another terminal, broadcast the initial zero-referenced transform (`odom -> imu_link`):
   ```bash
   python3 src/pladypos/src/python_nodes/extras/imu_tf.py --ros-args -r __ns:=/usv5
   ```
3. Open `rviz2`, set the **Fixed Frame** to `odom`, and add a **TF** or **Axes** display referenced to `imu_link`.

### 4.3 Magnetometer Hard-Iron Calibration
If the magnetic environment changes or the sensor is remounted, run the 40-second calibration routine while drawing figure-8 patterns in the air:
```bash
python3 src/pladypos/src/python_nodes/extras/calibrate_mag.py --port /dev/ttyACM0 --seconds 40
```
This will compute the new hard-iron offsets (`mag_offset_x`, `mag_offset_y`, `mag_offset_z`) to update in `driver_imu.py`.

### 4.4 IMU Physical Axis Verification
To verify the physical orientation and gravitational reading on each axis:
```bash
python3 src/pladypos/src/python_nodes/extras/check_axes.py --port /dev/ttyACM0
```

---

## 5. Vehicle Characteristics (Physical Platform)

* **Dimensions:** 756 x 756 x 280 mm
* **Weight:** 25.0 kg
* **Payload:** 5.0 kg + water displacement
* **Maneuverability:** 4 thrusters in an X-configuration enable omnidirectional motion and dynamic positioning (DP) at arbitrary orientations.
* **Development Context:** The physical platform was designed by LABUST (see [PlaDyFleet Website](https://pladyfleet.fer.hr/pladyfleet/pladypos)). This ROS 2 package update was developed by the German Research Center for Artificial Intelligence (DFKI).

---

## 6. Telemetry & Control Vector Mappings

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