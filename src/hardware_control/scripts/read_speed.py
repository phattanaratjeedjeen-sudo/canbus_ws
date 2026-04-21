import can

bus = can.interface.Bus(
    interface='slcan', 
    channel='/dev/ttyACM1', 
    bitrate=500000)

canID = 0x04
data = [0xF7]
crc = (canID + sum(data)) & 0xFF
data = data + [crc]

with bus:
    msg = can.Message(
        arbitration_id=canID,
        data=data,
        is_extended_id=False
    )

    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"Communication Error: {e}")