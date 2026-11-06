# Pladypos USV: Guía de Usuario y Protocolo de Inicio Rápido (SOP)

Este documento es la **Guía Operativa de Usuario** con la arquitectura distribuida Servidor (`rawfie`) $\leftrightarrow$ Barco (`pladypos-usv5`) $\leftrightarrow$ Laptop (`GCS`):

- **Servidor `rawfie` (`10.250.253.1` / `10.0.10.218`):** Ejecuta la cámara cenital (`0354`), la detección AprilTag en local (`visual_tag_estimator.py` $\rightarrow$ `/usv5/visual_odom` a $25\text{ Hz}$) y el **Router Central de Zenoh** en el puerto `7447`.
- **Barco `usv5` (`10.0.10.35`):** Se conecta directo a `rawfie` (`tcp/10.0.10.218:7447`, sin necesidad de túnel reverso SSH), lee su propia IMU a $100\text{ Hz}$, ejecuta **a bordo** el filtrado `direct_usv_fusion.py` ($30\text{ Hz}$ $\rightarrow$ `/usv5/odometry/filtered`) y corre **a bordo** el controlador de Dinámica Inversa (`inverse_dynamics_8.launch.py`).
- **Tu Laptop (`GCS`):** Se conecta a `rawfie` (`tcp/10.250.253.1:7447`), envía el **joystick** (`/usv5/joy`) y muestra **RViz** en tiempo real con la pileta, la referencia en 8 y la trayectoria del barco.

---

## 🚀 Paso a Paso Operativo

### 🖥️ Terminal 1 (Tu Laptop GCS) — Conectar Zenoh al Servidor `rawfie`
```bash
ros2 run pladypos start_zenoh_gcs.sh
```
*(Conecta tu laptop como cliente Zenoh a `tcp/10.250.253.1:7447` en `rawfie`).*

---

### 🚢 Terminal 2 (SSH al Barco) — Core + Fusión a Bordo del Barco
1. Conéctate al barco (ya no necesitas `-R 7447:...`):
   ```bash
   ssh usv5
   ```
2. Dentro del barco, lanza el core (Arduino + Teleop + IMU 100 Hz + **Fusión 30 Hz a bordo**):
   ```bash
   ros2 launch pladypos core.launch.py
   ```

---

### 🔗 Terminal 3 (SSH al Barco) — Cliente Zenoh a Bordo hacia `rawfie`
1. En otra terminal hacia el barco:
   ```bash
   ssh usv5
   ```
2. Dentro del barco, conecta el cliente Zenoh a `rawfie` (`tcp/10.0.10.218:7447`):
   ```bash
   ros2 run pladypos start_zenoh_usv.sh
   ```

---

### 🎮 Terminal 4 (Tu Laptop GCS) — Joystick + RViz en Tiempo Real
En tu laptop, lanza el joystick y RViz:
```bash
ros2 launch pladypos gcs.launch.py
```
- Se abrirá **RViz** mostrando el perímetro de la pileta (cian), la **trayectoria en forma de 8 al 70% centrada** (roja), el **punto de inicio** (esfera verde en $(-0.5713, -1.2731)\text{ m}$) y el barco (`usv5_odom`).
- Usa el **joystick** desde tu laptop para llevar el barco hasta el punto de inicio.

---

### 🎯 Terminal 5 (SSH al Barco) — Lanzar el Controlador de Dinámica Inversa a Bordo
Cuando ya ubicaste el barco con el joystick:
1. *(Opcional)* Puedes dejar **RViz abierto** en tu laptop (`teleop_mixer.py` bloquea automáticamente el joystick mientras el controlador autónomo está activo, o puedes lanzar `ros2 launch pladypos gcs.launch.py use_joy:=false`).
2. Dentro de una terminal SSH en el barco (`ssh usv5`), lanza el controlador a bordo:
   ```bash
   ros2 launch pladypos inverse_dynamics_8.launch.py
   ```
   - El barco calculará a bordo a **$30\text{ Hz}$** el control de Dinámica Inversa tomando `/usv5/odometry/filtered` y su IMU, y en el **RViz de tu laptop** verás moverse la flecha dorada de referencia y la estela verde de la trayectoria seguida por el barco.
   - Al presionar `Ctrl + C` en el controlador, el barco frena los 4 motores en neutro (`1500 us`), guarda el CSV de la prueba y reactiva el joystick.

---

## 📷 Nota sobre el Servidor `rawfie`
En `rawfie` ya quedó configurado `~/ros2-camera-test/config/start_all_cameras.launch.py` para que el contenedor `ros2_streaming_container` ejecute automáticamente:
1. El driver de la cámara `0354` y `stream_guard.py`.
2. `visual_tag_estimator.py` en `localhost` ($\rightarrow$ `/usv5/visual_odom`).
3. `zenoh-bridge-ros2dds` escuchando en `tcp/0.0.0.0:7447`.

Si alguna vez reinician el servidor `rawfie`, basta con entrar por `ssh rawfie` y ejecutar únicamente la cámara `.204`:
```bash
cd ~/ros2-camera-test && ./204-start.bash
```
