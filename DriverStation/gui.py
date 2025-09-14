import imgui
from enum import Enum

CONTROL_BAR_HEIGHT = 300
TELEMETRY_PANEL_WIDTH = 400
SHOW_IMGUI_TEST_WINDOW = False

CONFIRM_POPOP_WIDTH = 300
CONFIRM_POPUP_HEIGHT = 120
POPUP_IMGUI_FLAGS=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE
LED_INDICATOR_IMGUI_FLAGS=imgui.COLOR_EDIT_NO_BORDER | imgui.COLOR_EDIT_NO_TOOLTIP | imgui.COLOR_EDIT_NO_INPUTS

class GuiAction:
    def __init__(self, text, callback):
        self.text = text
        self.callback = callback

    def execute(self):
        self.callback()

class Gui: 
    def __init__(self):
        self.current_action = None
        pass
    
    def led_indicator(self, text, enabled):
        r = 0
        g = 0
        b = 0

        if enabled:
            g = 1
        else:
            r = 1

        imgui.text(text)
        imgui.same_line()
        imgui.color_button(f"{text}-LED", r, g, b, 1, LED_INDICATOR_IMGUI_FLAGS, 50)

    def button_with_popup(self, text, action):
        if(imgui.button(text)):
            self.current_action = GuiAction(text, action)

    def draw_ctrl_bar(self, state):
        with imgui.begin_group():
            imgui.begin_child(
                "STATUS_LEDS", border=False,
                flags=imgui.WINDOW_NO_SCROLLBAR,
                height=90,
                width=300
            )
            with imgui.begin_group():
                self.led_indicator(" COMMS", state.is_robot_connected())
                self.led_indicator("RP2040", state.get_telemetry().is_rp2040_connected())

            imgui.same_line(spacing=50)

            with imgui.begin_group():
                self.led_indicator("JOYSTICK", state.get_gamepad().is_connected())
                self.led_indicator(" ENABLED", state.is_robot_enabled())
            imgui.end_child()

            with imgui.begin_group():
                imgui.text("ROBOT STATE")

                if state.is_robot_enabled():
                    if imgui.button("Disable Robot"):
                        state.disable_robot()
                else:
                    if imgui.button("Enable Robot"):
                        state.enable_robot()

                self.button_with_popup("Run Auto Dig", state.run_auto_dig)
                self.button_with_popup("Restart Code", state.restart_robot_code)
                self.button_with_popup("Reboot Pi", state.reboot_pi)
                self.button_with_popup("Reboot RP2040", state.reboot_rp2040)

            imgui.same_line(spacing=50)

            with imgui.begin_group():
                imgui.text(f"LINKAGE: {state.get_linkage_state()}")
                self.button_with_popup("Deploy Linkage", state.deploy_intake)
                self.button_with_popup("Set Manual Control", state.manual_control_intake)
                if imgui.button("Reset Encoder"):
                    self.current_action = GuiAction('Reset Encoder', state.reset_intake_encoder)

        imgui.same_line(spacing=50)

        with imgui.begin_group():
            imgui.text("CONSOLE")
            imgui.same_line(spacing=10)
            imgui.button("Clear")

            imgui.begin_child(
                "Child 2", border=True,
                flags=imgui.WINDOW_NO_SCROLLBAR
            )
            imgui.text_wrapped("This text will wrap around.\n[INFO] this is a log message")
            imgui.end_child()
        
    def draw_telemetry(self, ds_state):
        telemetry = ds_state.get_telemetry()
        imgui.text(f"Robot enabled: {telemetry.is_robot_enabled()}")
        imgui.text(f"Robot mode: {telemetry.get_robot_mode()}")
        imgui.text("")
        imgui.text(f"Intake angle: {telemetry.get_intake_pos()}")
        imgui.text(f"Intake height: ")
        imgui.text("")
        # FROM OLD ROBOT for picking up and dumping things
        imgui.text("Dump speed: XXX")
        imgui.text("Dump torque: XXX")
        imgui.text("")

        gamepad = ds_state.get_gamepad()
        left_stick = gamepad.get_left_stick()
        right_stick = gamepad.get_right_stick()
        left_trigger = gamepad.get_left_trigger()
        right_trigger = gamepad.get_right_trigger()

        imgui.text("Gamepad:")
        imgui.text(f"Left Stick: {left_stick[0]:.2f}, {left_stick[1]:.2f}")
        imgui.text(f"Right Stick: {right_stick[0]:.2f}, {right_stick[1]:.2f}")
        imgui.text(f"Left Trigger: {left_trigger:.2f}")
        imgui.text(f"Right Trigger: {right_trigger:.2f}")

        imgui.text("")

        with imgui.begin_group():
            imgui.text("Buttons")
            self.led_indicator("A", gamepad.get_button_a())
            self.led_indicator("B", gamepad.get_button_b())
            self.led_indicator("X", gamepad.get_button_x())
            self.led_indicator("Y", gamepad.get_button_y())

        imgui.same_line(100)

        with imgui.begin_group():
            imgui.text(" DPAD")
            self.led_indicator("   UP", gamepad.get_dpad_up())
            self.led_indicator(" DOWN", gamepad.get_dpad_down())
            self.led_indicator(" LEFT", gamepad.get_dpad_left())
            self.led_indicator("RIGHT", gamepad.get_dpad_right())

        imgui.same_line(230)
        with imgui.begin_group():
            imgui.text("BUMPERS")
            self.led_indicator("  LEFT", gamepad.get_left_bumper())
            self.led_indicator(" RIGHT", gamepad.get_right_bumper())


    def draw_confirmation_box(self, window_width, window_height):
        if self.current_action != None:
            imgui.open_popup("Action Confirmation")

            imgui.set_next_window_size(CONFIRM_POPOP_WIDTH, CONFIRM_POPUP_HEIGHT)
            imgui.set_next_window_position((window_width / 2) - (CONFIRM_POPOP_WIDTH  / 2), (window_height / 2) - (CONFIRM_POPUP_HEIGHT / 2))
            with imgui.begin_popup_modal("Action Confirmation", flags=POPUP_IMGUI_FLAGS) as select_popup:
                if select_popup.opened:
                    imgui.text("Are you sure:")
                    imgui.text("")
                    if imgui.button(self.current_action.text):
                        self.current_action.execute()
                        self.current_action = None
                    imgui.same_line()
                    if imgui.button("Cancel"):
                        self.current_action = None

    def draw_robot(self, ds_state):
        """
            Robot GUI to show the speed and direction of each wheel
            Initial Goal:
                Have a robot animation, arrows on each motor detailing direction,
                arrows switch direction based of the speed (+/-), speed of motors
                next to each motor (there's 2 motors total)
            
            Stretch Goal:
                Have an arrow in front of the robot showing the direction based on the
                combined direction and speed of the 2 motors.

            TO-DO: look into the HAL and find the individual wheel speed
        """

        # Get motor speeds from the telemetry object
        telemetry = ds_state.get_telemetry()
        left_speed = telemetry.get_left_motor_speed()
        right_speed = telemetry.get_right_motor_speed()

        # Get the draw list for custom rendering
        draw_list = imgui.get_window_draw_list()
        
        # Window position for relative drawing
        pos = imgui.get_cursor_screen_pos()
        win_width = imgui.get_window_width()
        
        # Robot dimensions and positioning
        robot_width = 150
        robot_height = 200
        wheel_width = 30
        wheel_height = 60
        
        center_x = pos.x + win_width / 2
        top_y = pos.y + 50
        
        # --- Draw Robot Body ---
        # A simple rectangle for the chassis
        chassis_col = imgui.get_color_u32_rgba(0.5, 0.5, 0.5, 1)
        draw_list.add_rect_filled(
            center_x - robot_width / 2, top_y,
            center_x + robot_width / 2, top_y + robot_height,
            chassis_col, 5 # rounded corners
        )
        
        # --- Helper function to draw wheels and arrows ---
        def draw_wheel_and_arrow(x, y, speed):
            # Wheel
            wheel_col = imgui.get_color_u32_rgba(0.2, 0.2, 0.2, 1)
            draw_list.add_rect_filled(x, y, x + wheel_width, y + wheel_height, wheel_col, 3)

            # Arrow Color
            arrow_col = imgui.get_color_u32_rgba(0.1, 1, 0.1, 1) # Green for forward

            # Arrow direction and position
            arrow_center_y = y + wheel_height / 2
            arrow_x = x + wheel_width / 2
            
            if speed > 0: # Forward
                p1 = (arrow_x, arrow_center_y - 15)
                p2 = (arrow_x - 7, arrow_center_y)
                p3 = (arrow_x + 7, arrow_center_y)
                draw_list.add_triangle_filled(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, arrow_col)
            elif speed < 0: # Backward
                arrow_col = imgui.get_color_u32_rgba(1, 0.1, 0.1, 1) # Red for backward
                p1 = (arrow_x, arrow_center_y + 15)
                p2 = (arrow_x - 7, arrow_center_y)
                p3 = (arrow_x + 7, arrow_center_y)
                draw_list.add_triangle_filled(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, arrow_col)
            
            # Speed text
            text_col = imgui.get_color_u32_rgba(1, 1, 1, 1)
            imgui.set_cursor_screen_pos((x, y + wheel_height + 5))
            imgui.text(f"{speed}")

        # --- Draw Left Wheel ---
        left_wheel_x = center_x - robot_width / 2 - wheel_width - 10
        left_wheel_y = top_y + (robot_height - wheel_height) / 2
        draw_wheel_and_arrow(left_wheel_x, left_wheel_y, left_speed)
        imgui.set_cursor_screen_pos((left_wheel_x - 10, left_wheel_y - 20))
        imgui.text("Left Motor")
        
        # --- Draw Right Wheel ---
        right_wheel_x = center_x + robot_width / 2 + 10
        right_wheel_y = top_y + (robot_height - wheel_height) / 2
        draw_wheel_and_arrow(right_wheel_x, right_wheel_y, right_speed)
        imgui.set_cursor_screen_pos((right_wheel_x - 10, right_wheel_y - 20))
        imgui.text("Right Motor")

        


                


    def render(self, state, window_size):
        window_width, window_height = window_size

        imgui.set_next_window_size(window_width, CONTROL_BAR_HEIGHT)
        imgui.set_next_window_position(0, window_height - CONTROL_BAR_HEIGHT)
        imgui.begin("Control Bar", flags=imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        self.draw_ctrl_bar(state)
        imgui.end()

        imgui.set_next_window_size(TELEMETRY_PANEL_WIDTH, window_height - CONTROL_BAR_HEIGHT)
        imgui.set_next_window_position(0, 0)
        imgui.begin("Telemetry", flags=imgui.WINDOW_NO_COLLAPSE | imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE)
        self.draw_telemetry(state)
        self.draw_robot(state)
        imgui.end()

        self.draw_confirmation_box(window_width, window_height)

        if SHOW_IMGUI_TEST_WINDOW:
            imgui.show_test_window()