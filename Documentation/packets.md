# Serial Packets for URC Robot

## Driver Station -> Robot Packets

Below is the format of a serial packet from driver station to robot.
Byte 1: 0xBE
Byte 2: 0xEF
Byte 3: message type (PACKET_HEARTBEAT or PACKET_GAMEPAD)
Byte 4: 0x00 = disabled, 0x01 = teleop enabled, 0x02 = auto enabled
Byte 5: 0x9C-0x64 (left stick Y -100 thru 100 2s complement byte)
Byte 6: 0x9C-0x64 (right stick Y -100 thru 100 2s complement byte)
Byte 7: 0x00 or 0x01 boolean left trigger state
Byte 8: 0x00 or 0x01 boolean right trigger state
Byte 9: 0x00 or 0x01 boolean A button state
Byte 10: 0x00 or 0x01 boolean B button state
Byte 11: 0x00 or 0x01 boolean dpad up state
Byte 12: 0x00 or 0x01 boolean dpad down state
Byte 13: 0x00 or 0x01 boolean left bumper state
Byte 14: 0x00 or 0x01 boolean right bumper state

## Robot -> Driver Station Packets

Below is the format of a serial packet from robot to driver station.
Byte 1: 0xBE
Byte 2: 0xEF
Byte 3: message type (atm will always be PACKET_HEARTBEAT)
Byte 3: 0x00 = disabled, 0x01 = teleop enabled, 0x02 = auto enabled
Byte 4: 0x00 or 0x01 reports if CAN controller is connected

I'd like in the future to look into reporting battery levels too

## Old CAN Controller Packets (Need to rework HAL probably)

Set drive speed to 50%:
BE EF 81 32 32 32 32 00 00 00 00 00 00

Set deploy to 20%: 
BE EF 82 20 00 00 00 00 00 00 00 00 00 

BE EF 82 50 00 00 00 00 00 00 00 00 00 

Set deploy to -20%:
BE EF 82 EC 00 00 00 00 00 00 00 00 00 

Set intake speed to 10%:
BE EF 83 10 00 00 00 00 00 00 00 00 00

BE EF 83 20 00 00 00 00 00 00 00 00 00

Set dump
BE EF 84 25 00 00 00 00 00 00 00 00 00

Set intake speed to 0%:
BE EF 83 00 00 00 00 00 00 00 00 00 00

Bootload Mode:
BE EF A0 00 00 00 00 00 00 00 00 00 00

