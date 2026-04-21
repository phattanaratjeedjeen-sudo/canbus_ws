#!/usr/bin/python3

import can


class MKS():
    def __init__(self, canID, bus=None):
        self.canID = canID

        if bus is None:
            self.bus = can.interface.Bus(
                interface='slcan', 
                channel='/dev/ttyACM1', 
                bitrate=500000)
            self.owns_bus = True
        else:
            self.bus = bus
            self.owns_bus = False

    def __del__(self):
        if getattr(self, 'owns_bus', False) and self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass

    def send_speed(self, speed, acc):
        if speed < 0:
            dir = 0x80 # CW
        else:
            dir = 0x00 # CCW
        speed = abs(speed)
        byte1 = 0xF6
        speed_high = (speed >> 8) & 0x0F
        byte2 = dir | speed_high
        byte3 = speed & 0xFF
        byte4 = max(0, min(acc, 255))  
        data = [self.canID, byte1, byte2, byte3, byte4]
        self.send_command(data[1:])

    def read_speed(self):
        byte1 = 0x32
        self.send_command([byte1])

    def read_position(self):
        byte1 = 0x30
        self.send_command([byte1])

    def stop_motor(self):
        byte1 = 0xF7
        self.send_command([byte1])


    def reset_motor(self):
        byte1 = 0x41   
        self.send_command([byte1])

    def send_command(self, data):
        crc = (self.canID + sum(data)) & 0xFF
        full_data = data + [crc]

        msg = can.Message(
            arbitration_id=self.canID,
            data=full_data,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            # print(f"Message sent successfully to ID {self.canID:02X}")
        except can.CanError as e:
            print(f"Message NOT sent to ID {self.canID:02X}: {e}")


# cmd = MKS(canID=0x01).send_speed(speed=10, acc=5)
# cmd = MKS(canID=0x06).stop_motor()
# cmd = MKS(canID=0x06).reset_motor()