# Especificaciones para detección de AprilTag

## Fuente de imagen

- **Protocolo:** stream MJPEG servido por HTTP (tipo `web_video_server` de ROS2)
- **Host:** `10.250.253.1`
- **Puerto:** `8083`
- **URL del stream:** `http://10.250.253.1:8083/stream?topic=/camera_0352/camera_0352/image_raw`
- **Endpoint de calibración de cámara:** `http://10.250.253.1:8083/calibration?camera=0352`
  - Devuelve JSON con `camera_matrix.data` (matriz intrínseca 3x3 en formato plano: fx, 0, cx, 0, fy, cy, 0, 0, 1), `width` y `height`.

## Tag a detectar

- **ID:** `285`
- **Familia:** `AprilTag36h11`
- **Lado del tag:** `0.25 m` (25 cm)

## Calibración de referencia (valores por defecto si el endpoint falla)

- fx = 1109.2076
- fy = 1114.4800
- cx = 1019.4268
- cy = 1033.4869
- Resolución: 2048 x 2048
