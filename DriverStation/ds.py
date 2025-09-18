import networking
import window
import gui
from enum import Enum
import time
import pygame
import struct
import camera

DATA_UNKNOWN="---"
JOYSTICK_DEADZONE=0.1

class RobotMode(Enum):
    TELEOP = 0
    AUTO_EXECAVATE = 1,

    def __str__(self):
        if(self == self.TELEOP):
            return "TeleOperated"
        elif(self == self.HEIGHT_CONTROL):
            return "Auto Execavate"  
        return None


class LinkageState(Enum):
    RETRACTED = 0
    HEIGHT_CONTROL = 1,
    MANUAL = 2

    def __str__(self):
        if(self == self.RETRACTED):
            return "Retracted"
        elif(self == self.HEIGHT_CONTROL):
            return "Height Control"
        elif(self == self.MANUAL):
            return "Manual"

class RobotTelemetry: 
    def __init__(self):
        self.reset()

    def reset(self):
        self.robot_enabled = DATA_UNKNOWN
        self.rp2040_connected = False
        self.intake_pos = DATA_UNKNOWN
        self.robot_mode = DATA_UNKNOWN
        self.left_motor_speed = 0 
        self.right_motor_speed = 0
        self.autonomous_mode = False

    def set_robot_mode(self, mode):
        self.robot_mode = mode

    def get_robot_mode(self):
        return self.robot_mode

    def set_robot_enabled(self, enabled):
        self.robot_enabled = enabled

    def is_robot_enabled(self):
        return self.robot_enabled
    
    def set_rp2040_connected(self, connected):
        self.rp2040_connected = connected

    def is_rp2040_connected(self):
        return self.rp2040_connected
    
    def set_intake_pos(self, pos):
        self.intake_pos = pos

    def get_intake_pos(self):
        return self.intake_pos
    
    def get_left_motor_speed(self):
        return self.left_motor_speed

    def get_right_motor_speed(self):
        return self.right_motor_speed
    
    def get_autonomous_mode(self):
        return self.autonomous_mode

class GamepadState: 
    def __init__(self):
        self.set_connected(False)

    def set_connected(self, connected):
        self.connected = connected
        self.left_stick_x = 0.0
        self.left_stick_y = 0.0
        self.right_stick_x = 0.0
        self.right_stick_y = 0.0
        self.dpad_up = False
        self.dpad_right = False
        self.dpad_left = False
        self.dpad_down = False
        self.button_a = False
        self.button_b = False
        self.button_x = False
        self.button_y = False
        self.left_bumper = False
        self.right_bumper = False
        self.left_trigger = 0.0
        self.right_trigger = 0.0
        self.autonomous_mode = False

    def is_connected(self):
        return self.connected

    def set_joysticks(self, left_x, left_y, right_x, right_y):
        if abs(left_x) < JOYSTICK_DEADZONE:
            left_x = 0
        if abs(left_y) < JOYSTICK_DEADZONE:
            left_y = 0
        if abs(right_x) < JOYSTICK_DEADZONE:
            right_x = 0
        if abs(right_y) < JOYSTICK_DEADZONE:
            right_y = 0

        self.left_stick_x = left_x
        self.left_stick_y = left_y
        self.right_stick_x = right_x
        self.right_stick_y = right_y

    def get_left_stick(self):
        return [self.left_stick_x, self.left_stick_y]
    
    def get_right_stick(self):
        return [self.right_stick_x, self.right_stick_y]
    
    def set_buttons(self, a, b, x, y):
        self.button_a = a
        self.button_b = b
        self.button_x = x
        self.button_y = y

    def get_button_x(self):
        return self.button_x
    
    def get_button_y(self):
        return self.button_y

    def get_button_a(self):
        return self.button_a

    def get_button_b(self):
        return self.button_b

    def set_bumpers(self, left, right):
        self.left_bumper = left
        self.right_bumper = right

    def get_left_bumper(self):
        return self.left_bumper
    
    def get_right_bumper(self):
        return self.right_bumper
    
    def set_triggers(self, left, right):
        self.left_trigger = left
        self.right_trigger = right
 
    def get_left_trigger(self):
        return self.left_trigger
    
    def get_right_trigger(self):
        return self.right_trigger
    
    def set_dpad(self, up, down, left, right):
        self.dpad_up = up
        self.dpad_down = down
        self.dpad_left = left
        self.dpad_right = right

    def get_dpad_up(self):
        return self.dpad_up
    
    def get_dpad_down(self):
        return self.dpad_down
    
    def get_dpad_left(self):
        return self.dpad_left
    
    def get_dpad_right(self):
        return self.dpad_right
    
    def autonomous_mode(self):
        return self.autonomous_mode

class DriverStationState: 
    def __init__(self):
        self.running = True
        self.robot_connected = False
        self.robot_enabled = False
        self.linkage_state = LinkageState.RETRACTED
        self.gamepad = GamepadState()
        self.telemetry = RobotTelemetry()

    def set_robot_connected(self, connected):
        self.robot_connected = connected

        if not connected:
            self.telemetry.reset()
    
    def is_robot_connected(self):
        return self.robot_connected
    
    def get_gamepad(self):
        return self.gamepad
    
    def get_telemetry(self):
        return self.telemetry

    def enable_robot(self):
        self.robot_enabled = True
        print("Robot enabled")

    def disable_robot(self):
        self.robot_enabled = False
        print("Robot disabled")

    def is_robot_enabled(self):
        return self.robot_enabled

    def reset_intake_encoder(self):
        print("Reset intake encoder")

    def run_auto_dig(self):
        print("Run auto dig")

    def restart_robot_code(self):
        print("Restart robot code")

    def reboot_pi(self):
        print("Reboot pi")
    
    def reboot_rp2040(self):
        print("Reboot RP2040")

    def get_linkage_state(self):
        return str(self.linkage_state)
    
    def deploy_intake(self):
        print("Deploy intake")
        self.linkage_state = LinkageState.HEIGHT_CONTROL

    def manual_control_intake(self):
        print("Set deploy to manual")
        self.linkage_state = LinkageState.MANUAL

    def shutdown(self):
        self.running = False

class DriverStationInput:
    def __init__(self, ds_state):
        self.joystick = None
        self.update(ds_state)

    def connect_joystick(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)

    def update(self, ds_state : DriverStationState):
        if self.joystick is None:
            self.connect_joystick()
        elif pygame.joystick.get_count() == 0:
            self.joystick.quit()
            self.joystick = None
            
        gamepad_state = ds_state.get_gamepad()
        if self.joystick is not None:
            gamepad_state.set_connected(True)
            gamepad_state.set_joysticks(
                self.joystick.get_axis(0),
                -self.joystick.get_axis(1),
                self.joystick.get_axis(3),
                -self.joystick.get_axis(4)
            )
            gamepad_state.set_triggers(
                self.joystick.get_axis(2) / 2.0 + 0.5,
                self.joystick.get_axis(5) / 2.0 + 0.5
            )
            dpad_x, dpad_y = self.joystick.get_hat(0)
            gamepad_state.set_dpad(
                dpad_y > 0,
                dpad_y < 0,
                dpad_x > 0,
                dpad_x < 0
            )
            gamepad_state.set_buttons(
                self.joystick.get_button(0),
                self.joystick.get_button(1),
                self.joystick.get_button(2),
                self.joystick.get_button(3),
            )
            gamepad_state.set_bumpers(
                self.joystick.get_button(4),
                self.joystick.get_button(5)
            )
        else:
            gamepad_state.set_connected(False)

class RobotCommunicator:
    def __init__(self):
        self.last_gamepad_packet = 0
        self.last_heartbeat = 0
        pass

    def send_gamepad_packet(self, gamepad : GamepadState, connection : networking.ConnectionManager):
        button_set1 = 0
        button_set2 = 0

        # bitset1 |= self.joystick.get_button(8) << 0  # Left Stick Press
        # bitset1 |= self.joystick.get_button(9) << 1  # Right Stick Press
        button_set1 |= gamepad.get_dpad_up() << 2  # D-pad up
        button_set1 |= gamepad.get_dpad_down() << 3  # D-pad down
        button_set1 |= gamepad.get_dpad_left() << 4  # D-pad right
        button_set1 |= gamepad.get_dpad_right() << 5  # D-pad left
        button_set1 |= gamepad.get_button_a() << 6  # A button
        button_set1 |= gamepad.get_button_b() << 7  # B button

        button_set2 |= gamepad.get_button_x() << 0  # X button
        button_set2 |= gamepad.get_button_y() << 1  # Y button
        # button_set2 |= joystick.get_axis(2)>10 << 2  # Left trigger, scaled down
        # button_set2 |= joystick.get_axis(5)>10 << 3  # Right Trigger, scaled down
        button_set2 |= gamepad.get_left_bumper() << 4  # Left Bumper
        button_set2 |= gamepad.get_right_bumper() << 5  # Right Bumper
        # button_set2 |= joystick.get_button(7) << 6  # Start button
        # button_set2 |= joystick.get_button(6) << 7  # Select button

        packet = struct.pack(
                    '!bbbbbBBbbbb',
                    0x02,  # Message type for controller status
                    int(gamepad.get_left_stick()[1] * 100),
                    int(gamepad.get_left_stick()[0] * 100),
                    int(gamepad.get_right_stick()[1] * 100),
                    int(gamepad.get_right_stick()[0] * 100),
                    button_set1,
                    button_set2,
                    int(gamepad.get_left_trigger() * 100),
                    int(gamepad.get_right_trigger() * 100), 0, 0,  # Future usage placeholders
                )
        
        connection.send_packet(packet)

        self.last_gamepad_packet = time.time()

    def send_heartbeat(self, ds_state: DriverStationState, connection):
        packet = struct.pack(
            '!bbbbbbbbbbb',
            0x01, 
            ds_state.is_robot_enabled(), 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        connection.send_packet(packet)
        self.last_heartbeat = time.time()

    def handle_packet(self, packet, ds_state : DriverStationState):
        packet_type = packet[0]
        telemetry = ds_state.get_telemetry()

        if packet_type == 0x01: # Heartbeat
            ds_state.get_telemetry().set_robot_enabled(packet[1] != 0)
            ds_state.get_telemetry().set_rp2040_connected(packet[2] != 0)
            ds_state.get_telemetry().set_robot_mode(RobotMode(packet[3]))
            # THIS IS A PLACEHOLDER VALUE
        elif packet_type == 0x70: # Motor Telemetry Packet
            # Unpack left speed from the first payload byte (packet[1])
            # The value is a signed 8-bit int (-128 to 127)
            left_speed = int.from_bytes([packet[1]], byteorder='little', signed=True)
            
            # Unpack right speed from the second payload byte (packet[2])
            right_speed = int.from_bytes([packet[2]], byteorder='little', signed=True)
            
            # Update the telemetry object directly
            telemetry.left_motor_speed = left_speed
            telemetry.right_motor_speed = right_speed
        elif packet_type == 0x03:
            pos = packet[1] | (packet[2] << 8) | (packet[3] << 16) | (packet[3] << 24) 
            pos /= 100
            ds_state.get_telemetry().set_intake_pos(pos)
            # Bad placeholder code
        elif packet_type == 0x02:
            telemetry.autonomous_mode = False
        else:
            print(f"Unknown packet type from robot: {packet_type}")

    def update(self, ds_state, connection: networking.ConnectionManager):
        if time.time() - self.last_gamepad_packet > 0.02:
            self.send_gamepad_packet(ds_state.get_gamepad(), connection)

        if time.time() - self.last_heartbeat > 0.1:
            self.send_heartbeat(ds_state, connection)

        packet = connection.get_next_packet()
        while packet is not None:
            self.handle_packet(packet, ds_state)
            packet = connection.get_next_packet()

class DriverStation:
    def __init__(self):
        self.window = window.Window()
        self.gui = gui.Gui()
        self.state = DriverStationState()
        self.connection_manager = networking.ConnectionManager()
        self.input = DriverStationInput(self.state)
        self.robot_communicator = RobotCommunicator()

    def run(self):
        while self.state.running:             
            self.state.set_robot_connected(self.connection_manager.is_connected())

            if(self.connection_manager.is_connected()):
                self.robot_communicator.update(self.state, self.connection_manager)

            self.input.update(self.state)
            self.window.render(self.state, self.gui)
            self.window.process_events(self.state)

        self.connection_manager.shutdown()

camera.start_camera_webserver()
ds = DriverStation()
print("Driver station created. Running application!")
ds.run()
