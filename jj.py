import serial
import time

def set_pos(id, pos):
    tx = bytearray([
        0x80 | id,
        (pos >> 7) & 0x7F,
        pos & 0x7F
    ])

    with serial.Serial('/dev/serial0', baudrate=9600, timeout=1) as ser:
        ser.write(tx)
        rx = ser.read(3)

    return rx

if __name__ == "__main__":
    servo_id = 1
    target_position = 512  # Example target position

    response = set_pos(servo_id, target_position)
    print("Response:", response)
