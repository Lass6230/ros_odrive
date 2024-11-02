import can
from enum import Enum
import struct
import time


# -- start load
import json
with open('src/ros_odrive/odrive_ros2_control/odrive_ros2_control/flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']
# -- end load

# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}


class ODriveAxisState(Enum):
    AXIS_STATE_UNDEFINED                     = 0
    AXIS_STATE_IDLE                          = 1
    AXIS_STATE_STARTUP_SEQUENCE              = 2
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE     = 3
    AXIS_STATE_MOTOR_CALIBRATION             = 4
    AXIS_STATE_ENCODER_INDEX_SEARCH          = 6
    AXIS_STATE_ENCODER_OFFSET_CALIBRATION    = 7
    AXIS_STATE_CLOSED_LOOP_CONTROL           = 8
    AXIS_STATE_LOCKIN_SPIN                   = 9
    AXIS_STATE_ENCODER_DIR_FIND              = 10
    AXIS_STATE_HOMING                        = 11
    AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12
    AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13
    AXIS_STATE_ANTICOGGING_CALIBRATION       = 14





bus = can.interface.Bus("can0", bustype="socketcan",bitrate=1000000)

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass
node_id = 0
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x01), # 0x07: Set_Axis_State
    # data=struct.pack('<I', 11), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))


path = 'axis0.watchdog_feed'
endpoint_id = endpoints[path]['id']
print(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
        is_extended_id=False
    ))
# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
    is_extended_id=False
))

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
    data=struct.pack('<I', 11), # 8: AxisState.CLOSED_LOOP_CONTROL
    is_extended_id=False
))
# time.sleep(0.3)

import time

t_end = time.time() + 15
while time.time() < t_end:
    path = 'axis0.watchdog_feed'


    # Convert path to endpoint ID
    endpoint_id = endpoints[path]['id']
    print(endpoint_id) # 542

    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
        data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
        is_extended_id=False
    ))
    time.sleep(0.2)
    # bus.send(can.Message(
    # arbitration_id=(node_id << 5 | 0x01), # 0x07: Set_Axis_State
    # # data=struct.pack('<I', 11), # 8: AxisState.CLOSED_LOOP_CONTROL
    # is_extended_id=False
    # ))
    # for msg in bus:
    #     if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
    #         error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
    #         if state == 1: # 8: AxisState.CLOSED_LOOP_CONTROL
    #             print("state: ",state)
    #             # break

bus.shutdown()

# done = 0
# while(1):
#     time.sleep(0.1)
#     for msg in bus:
#         if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
#             error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
#             if state == 1: # 8: AxisState.CLOSED_LOOP_CONTROL
#                 done = 1
#                 break

#     if(done == 1):
#         break          
#     path = 'axis0.watchdog_feed'


#     # Convert path to endpoint ID
#     endpoint_id = endpoints[path]['id']
#     print(endpoint_id)

#     bus.send(can.Message(
#         arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
#         data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
#         is_extended_id=False
#     ))



# done = 0
# time.sleep(0.2)
# while(1):
#     time.sleep(0.1)
#     for msg in bus:
#         if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
#             error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
#             if state == 1: # 8: AxisState.CLOSED_LOOP_CONTROL
#                 done = 1
#                 break

#     # if(done == 1):
#     #     break          
#     path = 'axis0.watchdog_feed'


#     # Convert path to endpoint ID
#     endpoint_id = endpoints[path]['id']

#     bus.send(can.Message(
#         arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
#         data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
#         is_extended_id=False
#     ))

