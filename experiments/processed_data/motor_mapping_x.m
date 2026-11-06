%% ========================================================================
%  PlaDyPos USV (usv5) - Configuracion en X de Propulsores (4 Motores)
%  Archivo: motor_mapping_x.m
%  Descripcion: Define que motor corresponde a cada canal, su ubicacion
%               fisica en la configuracion en X, el pin de Arduino Micro,
%               la matriz de asignacion de control (mixing) y la conversion
%               a senal PWM en microsegundos [1100, 1900] us.
%% ========================================================================

clear motor_config;

%% 1. Diagrama Fisico de la Configuracion en X (Vista Superior / Planta)
%
%                        PROA / FRENTE (+X Surge, u)
%                                   ^
%                                   |
%              [Motor 2: FL]        |        [Motor 1: FR]
%               Pin A1 (idx 2)      |         Pin A0 (idx 1)
%                    \              |              /
%                     \             |             /
%                      \      +-----+-----+      /
%                       \     |           |     /
%   BABOR (+Y Sway, v) <------+  USV 5    +------  ESTRIBOR (-Y Sway)
%                       /     |  PlaDyPos |     \
%                      /      +-----+-----+      \
%                     /             |             \
%                    /              |              \
%              [Motor 4: BL]        |        [Motor 3: BR]
%               Pin A3 (idx 4)      |         Pin A2 (idx 3)
%                                   |
%                        POPA / ATRAS (-X Surge)
%
%  Convencion de giro Yaw (psi / r): Positivo antihorario (CCW, +Z hacia arriba)

%% 2. Identificacion de cada Motor (1 a 4)
% En ROS 2 (/usv5/pwm_out y /usv5/motor_feedback, std_msgs/Float32MultiArray):
%   - Python / C++ usa indices 0 a 3: [data[0], data[1], data[2], data[3]]
%   - MATLAB usa indices 1 a 4:       [u(1),    u(2),    u(3),    u(4)   ]

motor_config.num_motors = 4;
motor_config.configuration = 'X-Configuration (Omnidirectional 3-DOF Planar)';

motor_config.motors(1).matlab_index = 1;
motor_config.motors(1).ros_index    = 0;
motor_config.motors(1).code         = 'FR';
motor_config.motors(1).name_en      = 'Front Right';
motor_config.motors(1).name_es      = 'Delantero Derecho (Proa Estribor)';
motor_config.motors(1).arduino_pin  = 'A0';
motor_config.motors(1).csv_col_us   = 'pwm_us_m1_FR';
motor_config.motors(1).csv_col_norm = 'pwm_norm_m1_FR';

motor_config.motors(2).matlab_index = 2;
motor_config.motors(2).ros_index    = 1;
motor_config.motors(2).code         = 'FL';
motor_config.motors(2).name_en      = 'Front Left';
motor_config.motors(2).name_es      = 'Delantero Izquierdo (Proa Babor)';
motor_config.motors(2).arduino_pin  = 'A1';
motor_config.motors(2).csv_col_us   = 'pwm_us_m2_FL';
motor_config.motors(2).csv_col_norm = 'pwm_norm_m2_FL';

motor_config.motors(3).matlab_index = 3;
motor_config.motors(3).ros_index    = 2;
motor_config.motors(3).code         = 'BR';
motor_config.motors(3).name_en      = 'Back Right';
motor_config.motors(3).name_es      = 'Trasero Derecho (Popa Estribor)';
motor_config.motors(3).arduino_pin  = 'A2';
motor_config.motors(3).csv_col_us   = 'pwm_us_m3_BR';
motor_config.motors(3).csv_col_norm = 'pwm_norm_m3_BR';

motor_config.motors(4).matlab_index = 4;
motor_config.motors(4).ros_index    = 3;
motor_config.motors(4).code         = 'BL';
motor_config.motors(4).name_en      = 'Back Left';
motor_config.motors(4).name_es      = 'Trasero Izquierdo (Popa Babor)';
motor_config.motors(4).arduino_pin  = 'A3';
motor_config.motors(4).csv_col_us   = 'pwm_us_m4_BL';
motor_config.motors(4).csv_col_norm = 'pwm_norm_m4_BL';

%% 3. Matriz de Mezcla / Asignacion en Configuracion X (Control Mixing)
% Segun teleop_mixer.py y usv_bridge.ino:
%   u_FR =  surge - sway - yaw
%   u_FL =  surge + sway + yaw
%   u_BR = -surge - sway + yaw
%   u_BL = -surge + sway - yaw
%
% Vector de entrada generalizada: tau_cmd = [surge; sway; yaw]
% Vector de motores sin saturar:  u_raw   = [u_FR; u_FL; u_BR; u_BL] = M_mix * tau_cmd

motor_config.M_mix = [
     1, -1, -1;   % Motor 1: FR (Front Right)
     1,  1,  1;   % Motor 2: FL (Front Left)
    -1, -1,  1;   % Motor 3: BR (Back Right)
    -1,  1, -1    % Motor 4: BL (Back Left)
];

%% 4. Conversion entre Comando Normalizado [-1, 1] y PWM [1100, 1900] us
% En serial_bridge.py y usv_bridge.ino:
%   pwm_us = 1500 + 400 * u_norm
%   - 1500 us: Neutro (motor detenido)
%   - 1900 us: Maximo empuje hacia adelante (u_norm = +1.0)
%   - 1100 us: Maximo empuje en reversa     (u_norm = -1.0)

motor_config.pwm_neutral_us = 1500;
motor_config.pwm_half_span  = 400;
motor_config.pwm_min_us     = 1100;
motor_config.pwm_max_us     = 1900;

norm_to_pwm_us = @(u_norm) 1500 + 400 * max(-1.0, min(1.0, u_norm));
pwm_us_to_norm = @(pwm_us) (pwm_us - 1500) / 400.0;

%% 5. Resumen en Consola de MATLAB
fprintf('====================================================================\n');
fprintf('  PlaDyPos USV5 - Mapeo de Motores en Configuracion X\n');
fprintf('====================================================================\n');
fprintf('  Motor | Indice ROS | Codigo | Ubicacion Fisica                  | Pin\n');
fprintf('--------+------------+--------+-----------------------------------+-----\n');
for i = 1:motor_config.num_motors
    m = motor_config.motors(i);
    fprintf('   M%d   |     %d      |   %s   | %-33s | %s\n', ...
        m.matlab_index, m.ros_index, m.code, m.name_es, m.arduino_pin);
end
fprintf('====================================================================\n');
fprintf('  Formula PWM: PWM [us] = 1500 + 400 * u_norm  (Rango: [1100, 1900] us)\n');
fprintf('====================================================================\n');

