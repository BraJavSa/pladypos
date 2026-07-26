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
    while True:
        if ser.in_waiting > 100:
            ser.reset_input_buffer()
            payload = bytearray()
            state = 0
            continue
            
        b = ser.read(1)
        if not b:
            continue
        val = b[0]

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
                if payload_type == 0x01 and len(payload) == 12:
                    channels = struct.unpack('<6H', payload)
                    # Limit output to 20 Hz to avoid terminal print lag
                    now = time.time()
                    if now - last_print > 0.05:
                        last_print = now
                        sys.stdout.write(f"\rCh1: {channels[0]} | Ch2: {channels[1]} | Ch3: {channels[2]} | Ch4: {channels[3]}    ")
                        sys.stdout.flush()
            state = 0
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    ser.close()
