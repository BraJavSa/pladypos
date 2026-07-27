import serial
import struct
import time
import sys

port = '/dev/ttyACM0'
baud = 115200

print(f"Abriendo puerto {port}...")
try:
    ser = serial.Serial(port, baud, timeout=0)
    ser.reset_input_buffer()
except Exception as e:
    print(f"Error al abrir puerto: {e}")
    sys.exit(1)

print("Leyendo datos... Mueve los controles de la radio. Presiona Ctrl+C para salir.")

last_print = 0

try:
    byte_buffer = bytearray()
    while True:
        in_waiting = ser.in_waiting
        if in_waiting > 2000:
            print(f"\nWarning: Serial input buffer backlog too high ({in_waiting} bytes). Resetting buffer.")
            ser.reset_input_buffer()
            byte_buffer = bytearray()
            time.sleep(0.01)
            continue
        elif in_waiting > 0:
            byte_buffer.extend(ser.read(in_waiting))
        else:
            time.sleep(0.001)

        while True:
            # Find the header sequence 0xFF 0xFF
            idx = byte_buffer.find(b'\xFF\xFF')
            if idx == -1:
                # No header found. Keep the last byte if it's 0xFF, discard the rest
                if len(byte_buffer) > 0 and byte_buffer[-1] == 0xFF:
                    byte_buffer = byte_buffer[-1:]
                else:
                    byte_buffer = bytearray()
                break
            
            # If we found a header, make sure we have type and len
            if idx + 4 > len(byte_buffer):
                byte_buffer = byte_buffer[idx:]
                break

            payload_type = byte_buffer[idx + 2]
            payload_len = byte_buffer[idx + 3]
            
            # Check if we have the full payload and checksum
            total_len = idx + 4 + payload_len + 1
            if total_len > len(byte_buffer):
                byte_buffer = byte_buffer[idx:]
                break

            # Extract payload and checksum
            payload = byte_buffer[idx + 4 : idx + 4 + payload_len]
            checksum_val = byte_buffer[idx + 4 + payload_len]

            # Verify checksum
            expected = (payload_type + payload_len + sum(payload)) & 0xFF
            if expected == checksum_val:
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
                
                # Consume this packet
                byte_buffer = byte_buffer[total_len:]
            else:
                # Checksum failed, discard only the first 0xFF byte to look for next header
                byte_buffer = byte_buffer[idx + 1:]
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    ser.close()
