import serial
import struct
import time
import sys

port = '/dev/ttyACM0'
baud = 115200

print(f"Abriendo puerto {port}...")
try:
    ser = serial.Serial(port, baud, timeout=0.1)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error al abrir puerto: {e}")
    sys.exit(1)

state = 0
payload_type = 0
payload_len = 0
payload = bytearray()

print("Leyendo datos... Mueve los controles de la radio. Presiona Ctrl+C para salir.")

last_print = 0

try:
    byte_buffer = bytearray()
    while True:
        in_waiting = ser.in_waiting
        if in_waiting > 100:
            ser.reset_input_buffer()
            byte_buffer = bytearray()
            state = 0
            time.sleep(0.01)
            continue
        elif in_waiting > 0:
            byte_buffer.extend(ser.read(in_waiting))
        else:
            byte_buffer.extend(ser.read(1))

        while len(byte_buffer) > 0:
            val = byte_buffer.pop(0)
            if state == 0:
                if val == 0xFF:
                    state = 1
            elif state == 1:
                if val == 0xFF:
                    state = 2
                else:
                    state = 0
            elif state == 2:
                payload_type = val
                state = 3
            elif state == 3:
                payload_len = val
                payload = bytearray()
                state = 4
            elif state == 4:
                payload.append(val)
                if len(payload) == payload_len:
                    state = 5
            elif state == 5:
                checksum = (payload_type + payload_len + sum(payload)) & 0xFF
                if checksum == val:
                    now = time.time()
                    if now - last_print > 0.05:
                        last_print = now
                        if payload_type == 0x01:
                            if len(payload) == 14:
                                data_unpacked = struct.unpack('<7H', payload)
                                channels = data_unpacked[:6]
                                freq = data_unpacked[6]
                                sys.stdout.write(f"\r[Radio 0x01] Ch1: {channels[0]} | Ch2: {channels[1]} | Ch3: {channels[2]} | Freq: {freq} Hz      ")
                            else:
                                sys.stdout.write(f"\r[Radio 0x01] Got unexpected len {len(payload)}                      ")
                        elif payload_type == 0x02:
                            voltage = struct.unpack('<f', payload)[0]
                            sys.stdout.write(f"\r[Telemetry 0x02] Voltage: {voltage:.2f} V                      ")
                        elif payload_type == 0x03:
                            counter = struct.unpack('<H', payload)[0]
                            sys.stdout.write(f"\r[Counter 0x03] Val: {counter}                                  ")
                        else:
                            sys.stdout.write(f"\r[Packet {payload_type:#x}] Len: {payload_len}                  ")
                        sys.stdout.flush()
                state = 0
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    ser.close()
