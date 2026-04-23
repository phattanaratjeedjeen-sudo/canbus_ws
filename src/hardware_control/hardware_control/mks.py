import can
from can import BusState
import time
import numpy as np

class MKS():
    def __init__(self):
        self.dir = {
            -1: 0x80, # CW
             1: 0x00  # CCW
        }

        try:
            self.bus = can.interface.Bus(
                interface='slcan', 
                channel='/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209C36A83945-if00', 
                bitrate=500000)
            
            try:
                self.bus.state = BusState.PASSIVE
            except NotImplementedError:
                pass
            
            print("Connected to CAN bus via Openlight CANable2")
        except can.CanError as e:
            print(f"Failed to connect to CAN bus: {e}")

    def send_speed(self, canID, speed, acc):
        code = 0xF6
        speedH = (abs(speed) >> 8) & 0x0F
        dir_speedH = self.dir.get(np.sign(speed), 0x00) | speedH
        speedL = abs(speed) & 0xFF
        self.send_command(canID, [code, dir_speedH, speedL, max(0, min(acc, 255))])

    def read_speed(self, canID):
        code = 0x32
        self.send_command(canID, [code])

        timeout = 0.005
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(0.001)
            if msg is not None and msg.arbitration_id == canID and msg.data[0] == code:
                rmp = int.from_bytes(msg.data[1:3], byteorder='big', signed=True)
                return rmp
        return None

    def read_position(self, canID):
        code = 0x30
        self.send_command(canID,[code])
        
        timeout = 0.005
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.bus.recv(0.001)
            if msg is not None and msg.arbitration_id == canID and msg.data[0] == code:
                carry = int.from_bytes(msg.data[1:5], byteorder='big', signed=True)
                encoder = int.from_bytes(msg.data[5:7], byteorder='big', signed=False)
                return carry, encoder
        return None

    def stop_motor(self, canID):
        code = 0xF7
        self.send_command(canID, [code])

    def reset_motor(self, canID):
        code = 0x41   
        self.send_command(canID, [code])

    def go2pos(self, canID, target_pos, speed, acc):
        code = 0xF5
        speedH = abs(speed) >> 8 & 0x0F
        speedL = abs(speed) & 0xFF
        target_b5 = target_pos >> 16 & 0xFF
        target_b6 = target_pos >> 8 & 0xFF
        target_b7 = target_pos & 0xFF
        self.send_command(canID, [code, speedH, speedL, max(0,min(acc,255)) & 0xFF, target_b5, target_b6, target_b7])

    def set_home_position(self, canID):
        code = 0x92
        self.send_command(canID, [code])
        time.sleep(0.1)

    def send_command(self,canID, data):
        crc = (canID + sum(data)) & 0xFF
        full_data = data + [crc]

        msg = can.Message(
            arbitration_id=canID,
            data=full_data,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Communication Error: {e}")