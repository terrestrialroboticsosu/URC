import networking 
import window
import gui
from enum import Enum
import time
import pygame
import struct
import camera
import signal
import sys

DATA_UNKNOWN="---"
JOYSTICK_DEADZONE=0.1

class RobotMode(Enum):
    TELEOP = 0
    AUTO_EXECAVATE = 1

    def __str__(self):
        if self == self.TELEOP:
            return "TeleOperated"
        elif self == self.AUTO_EXECAVATE:
            return "Auto Execavate"  
        return None

class LinkageState(Enum):
    RETRACTED = 0
    HEIGHT_CONTROL = 1
    MANUAL = 2

    def __str__(self):
        if self == self.RETRACTED:
            return "Retracted"
        elif self == self.HEIGHT_CONTROL:
            return "Height Control"
        elif self == self.MANUAL:
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
        self.arrived_at_target = False 
        self.arm_joint_angles = [0.0] * 6  
        self.arm_end_effector_pos = [0.0, 0.0, 0.0]  

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
    
    def at_target(self):
        return self.arrived_at_target

    # Arm-specific setters/getters
    def set_arm_joint_angles(self, angles):
        if len(angles) == 6:
            self.arm_joint_angles = angles
    
    def get_arm_joint_angles(self):
        return self.arm_joint_angles
    
    def set_arm_end_effector_pos(self, pos):
        if len(pos) == 3:
            self.arm_end_effector_pos = pos
            
    def get_arm_end_effector_pos(self):
        return self.arm_end_effector_pos

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
        self.at_target = False

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

    def at_target(self):
        return self.at_target

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

    # Arm control
    def calibrate_arm(self):
        print("Calibrating arm...")

    def stow_arm(self):
        print("Stowing arm...")

# ... keep all imports and class definitions above as-is ...

class DriverStationInput:
    def __init__(self, ds_state):
        self.joystick = None
        pygame.joystick.init()
        self.connect_joystick()

    def connect_joystick(self):
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print(f"Joystick connected: {self.joystick.get_name()}")
        else:
            self.joystick = None

    def update(self, ds_state: DriverStationState):  # ✅ only one argument
        gamepad_state = ds_state.get_gamepad()
        if self.joystick is None or pygame.joystick.get_count() == 0:
            gamepad_state.set_connected(False)
            return

        axes = [self.joystick.get_axis(i) for i in range(self.joystick.get_numaxes())]
        def axis(idx): return axes[idx] if idx < len(axes) else 0.0
        def deadzone(val): return 0 if abs(val) < JOYSTICK_DEADZONE else val

        left_x = deadzone(axis(0))
        left_y = deadzone(-axis(1))
        right_x = deadzone(axis(3))
        right_y = deadzone(-axis(4))
        left_trigger = (axis(2) / 2.0 + 0.5) if len(axes) > 2 else 0.0
        right_trigger = (axis(5) / 2.0 + 0.5) if len(axes) > 5 else 0.0

        gamepad_state.set_connected(True)
        gamepad_state.set_joysticks(left_x, left_y, right_x, right_y)
        gamepad_state.set_triggers(left_trigger, right_trigger)

        if self.joystick.get_numhats() > 0:
            dpad_x, dpad_y = self.joystick.get_hat(0)
            gamepad_state.set_dpad(dpad_y > 0, dpad_y < 0, dpad_x < 0, dpad_x > 0)

        num_buttons = self.joystick.get_numbuttons()
        def btn(idx): return self.joystick.get_button(idx) if idx < num_buttons else 0
        gamepad_state.set_buttons(btn(0), btn(1), btn(2), btn(3))
        gamepad_state.set_bumpers(btn(4), btn(5))

class RobotCommunicator:
    def __init__(self):
        self.last_send_time = 0

    def send_gamepad_packet(self, gamepad, connection):
        import struct
        button_set1 = 0
        button_set2 = 0

        button_set1 |= gamepad.get_dpad_up() << 2
        button_set1 |= gamepad.get_dpad_down() << 3
        button_set1 |= gamepad.get_dpad_left() << 4
        button_set1 |= gamepad.get_dpad_right() << 5
        button_set1 |= gamepad.get_button_a() << 6
        button_set1 |= gamepad.get_button_b() << 7

        button_set2 |= gamepad.get_button_x() << 0
        button_set2 |= gamepad.get_button_y() << 1
        button_set2 |= gamepad.get_left_bumper() << 4
        button_set2 |= gamepad.get_right_bumper() << 5

        packet = struct.pack(
            '!bbbbbBBbbbb',
            0x02,
            int(gamepad.get_left_stick()[1] * 100),
            int(gamepad.get_left_stick()[0] * 100),
            int(gamepad.get_right_stick()[1] * 100),
            int(gamepad.get_right_stick()[0] * 100),
            button_set1,
            button_set2,
            int(gamepad.get_left_trigger() * 100),
            int(gamepad.get_right_trigger() * 100),
            0, 0
        )
        connection.send_packet(packet)
        print("sent gamepad packet")

    def send_heartbeat(self, ds_state, connection):
        import struct
        packet = struct.pack(
            '!bbbbbbbbbbb',
            0x01,
            int(ds_state.is_robot_enabled()), 0, 0, 0, 0, 0, 0, 0, 0, 0
        )
        connection.send_packet(packet)
        print("sent heartbeat")

    def update(self, ds_state, connection):
        import time
        now = time.time()
        if now - self.last_send_time >= 1.0:  # every 1 second
            self.send_gamepad_packet(ds_state.get_gamepad(), connection)
            self.send_heartbeat(ds_state, connection)
            self.last_send_time = now

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
            self.input.update(self.state)   # <-- update joystick first
            self.robot_communicator.update(self.state, self.connection_manager)
            self.window.render(self.state, self.gui)
            self.window.process_events(self.state)
            pygame.time.wait(5)



ds = DriverStation()
print("Driver station created. Running application!")

def signal_handler(sig, frame):
    print("Ctrl+C pressed, shutting down...")
    ds.state.shutdown()
    ds.connection_manager.shutdown()
    pygame.quit()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

try:
    ds.run()
finally:
    ds.connection_manager.shutdown()
    pygame.quit()
    print("Driver station exited cleanly.")
