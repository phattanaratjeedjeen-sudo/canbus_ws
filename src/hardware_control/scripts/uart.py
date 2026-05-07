import serial
import numpy as np

try:
    ser = serial.Serial(
        port='/dev/serial/by-id/usb-FTDI_USB__-__Serial-if00-port0',
        baudrate=38400,
        timeout=1
    )
except serial.SerialException as e:
    print(f"Failed to connect to serial port: {e}")

dir = {
    -1: 0x80, # CW
    1: 0x00  # CCW
}

Mstep = 16

def send_command(id, data):
    crc = (id + sum(data)) & 0xFF
    full_data = [id] + data + [crc]
    ser.write(full_data)

def send_speed(id, vrpm):
    code = 0xF6
    speed = int(200 * Mstep * abs(vrpm) / 30000)
    dir_speed = dir.get(np.sign(vrpm), 0x00) | (speed & 0x7F)
    send_command(id, [code, dir_speed])
    print(speed)

# def read_speed(id):
#     pass

# def read_abs_position(id):
#     pass

# def reset_motor(id):
#     pass

# def stop_motor(id):
#     pass

# def go2pos(id, target_pos, speed):
#     pass

send_speed(id=0xe3, vrpm=-1150)
