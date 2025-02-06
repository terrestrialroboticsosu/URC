import pygame
from pygame.locals import *
import struct
import socket
import time




def get_button_bitset():
    """Generate the bitset for buttons according to the specified mapping."""
    bitset = 0
    bitset |= joystick.get_button(8) << 0  # Left Stick Press
    bitset |= joystick.get_button(9) << 1  # Right Stick Press
    bitset |= 0 << 2  # D-pad up
    bitset |= 0 << 3  # D-pad down
    bitset |= 0 << 4  # D-pad right
    bitset |= 0 << 5  # D-pad left
    bitset |= joystick.get_button(0) << 6  # A button
    bitset |= joystick.get_button(1) << 7  # B button
    return bitset

def get_button_bitset2():
    """Generate the second bitset for buttons according to the specified mapping."""
    bitset = 0
    bitset |= joystick.get_button(2) << 0  # X button
    bitset |= joystick.get_button(3) << 1  # Y button
    bitset |= joystick.get_axis(2)>10 << 2  # Left trigger, scaled down
    bitset |= joystick.get_axis(5)>10 << 3  # Right Trigger, scaled down
    bitset |= joystick.get_button(4) << 4  # Left Bumper
    bitset |= joystick.get_button(5) << 5  # Right Bumper
    bitset |= joystick.get_button(7) << 6  # Start button
    bitset |= joystick.get_button(6) << 7  # Select button
    return bitset



def encode_gamepad_state():
    lastTime = 0
    while True:
        for event in pygame.event.get(): # get the events (update the joystick)
            if event.type == QUIT: # allow to click on the X button to close the window
                pygame.quit()
                exit()

        button_set_1 = get_button_bitset()
        button_set_2 = get_button_bitset2()

        packet = struct.pack(
                    '!bbbbbBBbbbb',
                    0x01,  # Message type for controller status
                    int(-joystick.get_axis(1)*100),
                    int(-joystick.get_axis(0)*100),
                    int(-joystick.get_axis(4)*100),
                    int(-joystick.get_axis(3)*100),
                    button_set_1,
                    button_set_2,
                    int((joystick.get_axis(5)+1)*50),
                    int((joystick.get_axis(2)+1)*50), 0, 0,  # Future usage placeholders
                )
        print("Packet:", packet.hex(' '))
        if(time.time() > lastTime + .1):
            s.send(packet)
            lastTime = time.time()

if __name__ == "__main__":
    try: 
        pygame.init()
        
        joystick = pygame.joystick.Joystick(0)
        TCP_IP = '192.168.0.10' 
        TCP_PORT = 2000

        while True:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((TCP_IP, TCP_PORT))
                break
            except: 
                print("Failed to connect. Sleep 1 second...")
                time.sleep(1)

        encode_gamepad_state()

    except KeyboardInterrupt:
        print("\nStopped listening for gamepad events.")