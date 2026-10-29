# AprilTag Detection Specifications

## Image Source

- **Protocol:** HTTP MJPEG stream (ROS 2 `web_video_server`)
- **Host:** `10.250.253.1`
- **Port:** `8083`
- **Stream URL:** `http://10.250.253.1:8083/stream?topic=/camera_0352/camera_0352/image_raw`
- **Camera Calibration Endpoint:** `http://10.250.253.1:8083/calibration?camera=0352`
  - Returns JSON containing `camera_matrix.data` (3x3 intrinsic matrix in flat format: fx, 0, cx, 0, fy, cy, 0, 0, 1), `width`, and `height`.

## Target Tag

- **ID:** `285`
- **Family:** `AprilTag36h11`
- **Tag Side Length:** `0.25 m` (25 cm)

## Reference Calibration (Default Fallback Parameters)

- fx = 1109.2076
- fy = 1114.4800
- cx = 1019.4268
- cy = 1033.4869
- Resolution: 2048 x 2048
