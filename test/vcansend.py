import time
import can 

bustype = 'socketcan'
channel = 'vcan0'

def producer(id):
    """:param id: Spam the bus with messages including the data id."""
    bus = can.Bus(channel=channel, interface=bustype)
    for i in range(1):
        msg = can.Message(arbitration_id=0x386, data=[id, 1, 1, 1, 1, 1, 1, 1], is_extended_id=False)
        bus.send(msg)

    time.sleep(1)

producer(10)
