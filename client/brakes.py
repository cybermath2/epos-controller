import can
import os

os.system('sudo ifconfig can0 down')
os.system('sudo ip link set can0 type can bitrate 1000000')  # 1mBit/s, should be the same value as in the Arduino
os.system('sudo ifconfig can0 up')


with can.interface.Bus(channel='can0', bustype='socketcan') as bus:
    msg = can.Message(arbitration_id=0x602, is_extended_id=False, dlc=8, data=[0,0,0,0,
                                                                               0,0,0,0,])
    
    try:
        bus.send(msg)
    except can.CanError:
        print("err")