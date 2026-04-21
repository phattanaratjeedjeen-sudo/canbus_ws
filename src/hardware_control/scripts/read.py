#!/usr/bin/env python

"""
Shows how to receive messages via polling.
"""

import can
from can.bus import BusState

bus = can.interface.Bus(interface='slcan', channel='/dev/ttyACM1', bitrate=500000)

def parse_position(msg):
    # ตรวจสอบว่า Message มีความยาวพอ และเป็นคำสั่ง 0x30 (Read Position)
    if msg.dlc >= 8 and msg.data[0] == 0x30:
        
        # ดึงเฉพาะ Byte ที่ 1 ถึง 4 (หรือ 2 ถึง 5 ขึ้นอยู่กับสเปก MKS รุ่นนั้น) 
        # สมมติว่าค่า Position ขนาด 32-bit (4 Bytes) อยู่ที่ Byte 1-4
        position_bytes = msg.data[1:5] 
        
        # แปลงเป็น Integer ทันที (รองรับค่าติดลบด้วย signed=True)
        position = int.from_bytes(position_bytes, byteorder='big', signed=True)
        
        print(f"Motor ID {msg.arbitration_id} -> Position: {position}")
        return position
        
    return None

# สมมติว่าจำลองข้อมูลจาก Log ของคุณ
# msg = can.Message(arbitration_id=1, data=bytearray(b'\x30\x00\x00\x00\x00\x00\x00\x31'))
# parse_position(msg) 

def receive_all():
    """Receives all messages and prints them to the console until Ctrl+C is pressed."""

    # this uses the default configuration (for example from environment variables, or a
    # config file) see https://python-can.readthedocs.io/en/stable/configuration.html
    with bus:
        # set to read-only, only supported on some interfaces
        try:
            bus.state = BusState.PASSIVE
        except NotImplementedError:
            pass

        try:
            while True:
                msg = bus.recv(1)
                if msg is not None:
                    print(msg.arbitration_id)
                    print(msg.data)
                    print(msg)
                    print("----")

                    parse_position(msg)
                    """
                    
                    
                    Example Output:
                    from emerstop
                    1 --> id
                    bytearray(b'\xf6\x00\xf7') -->  data
                    2
                    bytearray(b'\xf6\x00\xf8')
                    4
                    bytearray(b'\xf6\x01\xfb')

                    
                    from send_speed
                    bytearray(b'\xf6\x00\xf7')
                    4
                    bytearray(b'\xf6\x01\xfb')
                    2
                    bytearray(b'\xf6\x00\xf8')
                    """

                    
        except KeyboardInterrupt:
            pass  # exit normally


if __name__ == "__main__":
    receive_all()

