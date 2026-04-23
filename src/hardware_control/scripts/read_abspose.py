import can
from can.bus import BusState
import time

bus = can.interface.Bus(
    interface='slcan', 
    channel='/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209C36A83945-if00', 
    bitrate=500000)

try:
    bus.state = BusState.PASSIVE
except NotImplementedError:
    pass

canID = 0x05
data = [0x31]

def send_command(canID, data):
    crc = (canID + sum(data)) & 0xFF
    full_data = data + [crc]

    msg = can.Message(
        arbitration_id=canID,
        data=full_data,
        is_extended_id=False
    )
    try:
        bus.send(msg)

    except can.CanError as e:
        print(f"Communication Error: {e}")


while True:
    send_command(canID, data)
    time.sleep(1.0)
    msg = bus.recv(timeout=1.0)
    if msg is not None and msg.arbitration_id == canID and msg.data[0] == data[0]:
        pos_data = msg.data[1:7] 
        pos = int.from_bytes(pos_data, byteorder='big', signed=True)

        print(f"encoder: {pos}")

