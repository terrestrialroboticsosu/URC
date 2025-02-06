import struct
from inputs import get_gamepad
import socket
import time

def get_button_bitset(buttons):
    """Generate the bitset for buttons according to the specified mapping."""
    bitset = 0
    bitset |= buttons.get('BTN_THUMBL', 0) << 0  # Left Stick Press
    bitset |= buttons.get('BTN_THUMBR', 0) << 1  # Right Stick Press
    bitset |= buttons.get('BTN_DPAD_UP', 0) << 2  # D-pad up
    bitset |= buttons.get('BTN_DPAD_DOWN', 0) << 3  # D-pad down
    bitset |= buttons.get('BTN_DPAD_RIGHT', 0) << 4  # D-pad right
    bitset |= buttons.get('BTN_DPAD_LEFT', 0) << 5  # D-pad left
    bitset |= buttons.get('BTN_SOUTH', 0) << 6  # A button
    bitset |= buttons.get('BTN_EAST', 0) << 7  # B button
    return bitset

def get_button_bitset2(buttons):
    """Generate the second bitset for buttons according to the specified mapping."""
    bitset = 0
    bitset |= buttons.get('BTN_WEST', 0) << 0  # X button
    bitset |= buttons.get('BTN_NORTH', 0) << 1  # Y button
    bitset |= buttons.get('ABS_Z', 0) >> 7 << 2  # Left trigger, scaled down
    bitset |= buttons.get('ABS_RZ', 0) >> 7 << 3  # Right Trigger, scaled down
    bitset |= buttons.get('BTN_TL', 0) << 4  # Left Bumper
    bitset |= buttons.get('BTN_TR', 0) << 5  # Right Bumper
    bitset |= buttons.get('BTN_START', 0) << 6  # Start button
    bitset |= buttons.get('BTN_SELECT', 0) << 7  # Select button
    return bitset

def encode_gamepad_state():
    buttons = {}  # Initialize button states
    joystick_values = {
        'ABS_X': 0, 'ABS_Y': 0, 'ABS_RX': 0, 'ABS_RY': 0,
    }
    
    while True:
        events = get_gamepad()
        for event in events:
            # Update button or joystick state
            if event.ev_type == 'Key' or event.ev_type == 'Absolute':
                buttons[event.code] = event.state
            if event.code in joystick_values:
                joystick_values[event.code] = event.state
                print(joystick_values['ABS_Y'])

            # Once all values updated, encode packet
            if event.ev_type == 'Sync':
                button_set_1 = get_button_bitset(buttons)
                button_set_2 = get_button_bitset2(buttons)
                packet = struct.pack(
                    '!hhhhhhhhhhh',
                    0x01,  # Message type for controller status
                    joystick_values['ABS_Y'],
                    joystick_values['ABS_X'],
                    joystick_values['ABS_RY'],
                    joystick_values['ABS_RX'],
                    button_set_1,
                    button_set_2,
                    0, 0, 0, 0xA  # Future usage placeholders
                )
                print("Packet:", packet.hex())
                s.send(packet)
                time.sleep(1)



if __name__ == "__main__":
    try:
        TCP_IP = '127.0.0.1' 
        TCP_PORT = 2000      
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        encode_gamepad_state()
    except KeyboardInterrupt:
        print("\nStopped listening for gamepad events.")