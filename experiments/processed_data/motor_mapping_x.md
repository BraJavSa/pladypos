# Mapeo de Motores en Configuración X — PlaDyPos (`usv5`)

El vehículo autónomo de superficie **PlaDyPos (`usv5`)** cuenta con **4 propulsores (motores)** dispuestos en **configuración en X** en las cuatro esquinas del casco ($756 \times 756\text{ mm}$), permitiendo movimiento omnidireccional en el plano horizontal (Surge $u$, Sway $v$ y Guiñada/Yaw $r$).

---

## 1. Diagrama en Planta (Vista Superior — Configuración X)

```text
                       PROA / FRENTE (+X Surge, u)
                                  ^
                                  |
             [Motor 2: FL]        |        [Motor 1: FR]
              Pin A1 (idx 1)      |         Pin A0 (idx 0)
                   \              |              /
                    \             |             /
                     \      +-----+-----+      /
                      \     |           |     /
  BABOR (+Y Sway, v) <------+   USV 5   +------  ESTRIBOR (-Y Sway)
                      /     | PlaDyPos  |     \
                     /      +-----+-----+      \
                    /             |             \
                   /              |              \
             [Motor 4: BL]        |        [Motor 3: BR]
              Pin A3 (idx 3)      |         Pin A2 (idx 2)
                                  |
                       POPA / ATRÁS (-X Surge)
```

---

## 2. Tabla de Identificación de Motores

En los tópicos de ROS 2 (`/usv5/pwm_out` y `/usv5/motor_feedback`, tipo `std_msgs/msg/Float32MultiArray`), el vector de control tiene 4 elementos ordenados de la siguiente manera:

| Motor | Índice Python/ROS (`0..3`) | Índice MATLAB (`1..4`) | Código | Ubicación Física | Pin Arduino Micro | Columna PWM (`[us]`) en CSV | Columna Normalizada (`[-1, 1]`) en CSV |
| :---: | :---: | :---: | :---: | --- | :---: | :---: | :---: |
| **Motor 1** | `0` | `1` | **`FR`** | **Front Right** (Delantero Derecho / Proa Estribor) | `A0` | `pwm_us_m1_FR` | `pwm_norm_m1_FR` |
| **Motor 2** | `1` | `2` | **`FL`** | **Front Left** (Delantero Izquierdo / Proa Babor) | `A1` | `pwm_us_m2_FL` | `pwm_norm_m2_FL` |
| **Motor 3** | `2` | `3` | **`BR`** | **Back Right** (Trasero Derecho / Popa Estribor) | `A2` | `pwm_us_m3_BR` | `pwm_norm_m3_BR` |
| **Motor 4** | `3` | `4` | **`BL`** | **Back Left** (Trasero Izquierdo / Popa Babor) | `A3` | `pwm_us_m4_BL` | `pwm_norm_m4_BL` |

---

## 3. Ecuaciones de Mezcla en Configuración X (`teleop_mixer.py` / `usv_bridge.ino`)

Dado un comando de esfuerzo en el marco del cuerpo $(\text{surge}, \text{sway}, \text{yaw})$:

$$\begin{bmatrix} u_{1,\text{FR}} \\ u_{2,\text{FL}} \\ u_{3,\text{BR}} \\ u_{4,\text{BL}} \end{bmatrix} = \begin{bmatrix} +1 & -1 & -1 \\ +1 & +1 & +1 \\ -1 & -1 & +1 \\ -1 & +1 & -1 \end{bmatrix} \begin{bmatrix} \text{surge} \\ \text{sway} \\ \text{yaw} \end{bmatrix}$$

Si $\max_i |u_i| > 1.0$, el vector se normaliza dividiendo por $\max_i |u_i|$ para evitar saturación y preservar la dirección del esfuerzo resultante.

---

## 4. Conversión a Señal PWM por Motor (`serial_bridge.py` / `usv_bridge.ino`)

Cada comando normalizado $u_i \in [-1.0, +1.0]$ se convierte al ancho de pulso PWM en microsegundos enviado al ESC de cada propulsor:

$$\text{PWM}_i \, [\mu\text{s}] = 1500 + 400 \cdot u_i \quad \in [1100, \, 1900]\,\mu\text{s}$$

- **`1500 us`** ($u_i = 0.0$): Neutro (propulsor detenido).
- **`1900 us`** ($u_i = +1.0$): Máximo empuje hacia adelante.
- **`1100 us`** ($u_i = -1.0$): Máximo empuje en reversa.

