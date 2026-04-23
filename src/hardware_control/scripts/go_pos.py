import can
import time 


try:
    bus = can.interface.Bus(
        interface='slcan', 
        channel='/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209C36A83945-if00', 
        bitrate=500000)
    
    try:
        bus.state = can.bus.BusState.PASSIVE
    except NotImplementedError:
        pass

except can.CanError as e:
    print(f"Failed to connect to CAN bus: {e}")

def calibrate(canID):
    code = 0x40
    send_command(canID, [code])

def read_abspose(canID):
    code = 0x31
    send_command(canID, [code])

    timeout = 0.005
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(0.001)
        if msg is not None and msg.arbitration_id == canID and msg.data[0] == code:
             pos_data = msg.data[1:7] 
             pos = int.from_bytes(pos_data, byteorder='big', signed=True)
             return pos
    return None

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

# calibrate(0x06)

while True:
    # go2pos(canID=0x02, target_pos=0, speed=100, acc=5)
    time.sleep(1.0)
    pos = read_abspose(0x02)
    if pos is not None:
        print(f"Current Position: {pos}")

# a = 2500
# print(f"{a:02X}")           # hex format
# print(f"{a:b}")             # binary format    
# print(f"{a & 0xFF:b}")
# print(f"{a & 0xFF:X}")
# print(a & 0xFF)