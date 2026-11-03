# USV SSH Connection & SOCKS5 Proxy Guide

This guide describes how to connect to the USV onboard computer (`pladypos-usv5`) from the GCS and route Internet traffic (for `git pull` and `apt install`) through an SSH reverse SOCKS5 proxy tunnel.

---

## 1. Connect via SSH

### A. Direct Wired Connection (via rawfie ProxyJump - No WiFi required)
The camera master server `rawfie` (`10.250.253.1` on `bond0`) is directly connected via Ethernet `eno2` (`10.0.10.218`) to the boat's ASUS router network. You can connect from your GCS over the wired lab network using SSH ProxyJump:

```bash
# Direct command with reverse SOCKS5 proxy:
ssh -J rawfie -R 8888 pladypos-usv5@10.0.10.35
```

Or using the configured alias in `~/.ssh/config`:
```bash
ssh -R 8888 usv5
```
* **Password:** `labos123`
* Port `8888` on the USV is now bound to your local machine's proxy tunnel (`127.0.0.1:8888`).

### B. Direct Connection via Vehicle WiFi
If your GCS is connected directly to the boat's WiFi AP (`10.0.10.0/16`):
```bash
ssh -R 8888 pladypos-usv5@10.0.10.35
```
* **Password:** `labos123`

---

## 2. Update Codebase (`git pull`) via SOCKS5 Proxy

On the USV terminal:
```bash
cd ~/ros2usv_ws/src/pladypos
git -c http.proxy=socks5h://127.0.0.1:8888 pull
```

---

## 3. Install System / ROS 2 Packages (`apt`) via SOCKS5 Proxy

If the USV needs to install new dependencies (such as `imu_filter_madgwick`):

```bash
# 1. Update repositories through the SOCKS5 proxy
sudo apt update -o Acquire::http::Proxy="socks5h://127.0.0.1:8888" -o Acquire::https::Proxy="socks5h://127.0.0.1:8888"

# 2. Install the required ROS 2 package
sudo apt install -y -o Acquire::http::Proxy="socks5h://127.0.0.1:8888" -o Acquire::https::Proxy="socks5h://127.0.0.1:8888" ros-jazzy-imu-filter-madgwick
```

---

## 4. Pool Overhead Cameras Activation (`ssh rawfie`)

To use visual localization with AprilTag, the overhead cameras in the pool must be started from the camera master server:

1. **Connect to the Camera Server:**
   ```bash
   ssh rawfie
   ```
   > **Access Requirement:** Your PC must have an authorized SSH key configured, or you must consult the laboratory administrator for the password.

2. **Launch the Camera Stream:**
   Once connected inside `rawfie`, launch the overhead camera streaming service (`web_video_server`). The MJPEG stream will be available on the network at `http://10.250.253.1:8083` (see [apriltag_specs.md](file:///home/brayan/ros2_ws/src/pladypos/src/python_nodes/apriltag_specs.md) for endpoint and calibration specifications).

---

## 5. Remmina GUI Connection via SSH Tunnel (Over Wired Network)

To connect to the boat using **Remmina** without being connected to WiFi:

1. Open **Remmina** and edit your boat profile (`pladypos-usv5`).
2. Go to the **SSH Tunnel** tab:
   - Check **Enable SSH tunnel**.
   - Under **Custom**, enter: `10.250.253.1:22` (or `rawfie:22`).
   - **Authentication type:** `Password` (or `Public key (automatic)`).
   - **Username:** `uw`
   - **Password:** Password for user `uw` on `rawfie`.
3. Go to the **Basic** tab:
   - **Server:** `10.0.10.35`
   - **Username:** `pladypos-usv5`
4. Click **Save and Connect**. Remmina will route the connection over the laboratory wire through `rawfie` directly to the boat.

