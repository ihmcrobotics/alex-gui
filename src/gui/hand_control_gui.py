from communication.messages import EZGripperCommand, EZGripperState
from .gui_helpers import *

# Set the names of the hands
hand_names = ["left_hand", "right_hand"]

# Set the names of the operation modes and the dictionary for them
desired_operation_suffix = "_desired_op_mode"
CALIBRATION = "Calibration"
POSITION_CONTROL = "Position Control"
ERROR_RESET = "Error Reset"
hand_operation_modes = {POSITION_CONTROL: 0,
                        CALIBRATION: 1,
                        ERROR_RESET: 2}


class HandControlGUI:
    """
    This class creates the GUI for controlling the EZGripper hands
    """

    def __init__(self, monitor_scale: float = 1.0):
        """
        Initialize the hand control GUI
        :param monitor_scale: Scaling for dearpygui items based on the monitor being used
        """

        # Create and populate the hand control window
        with dpg.window(label="Hand Control", tag="hand_control_gui"):
            dpg.add_text("Hand Initialization: ")
            with dpg.group(horizontal=True):
                dpg.add_button(label="Calibrate Hands", callback=self.calibrate_hands)
                dpg.add_button(label="Reset Errors", callback=self.reset_errors)
            dpg.add_text("Hand Operation")
            with dpg.group(horizontal=True):
                dpg.add_button(label="Enable Hands", callback=self.enable_hands)
                dpg.add_button(label="Disable Hands", callback=self.disable_hands)
            dpg.add_text("Hand Positions:")
            with dpg.group(horizontal=True):
                dpg.add_button(label="Close Hands", callback=self._close_hands)
                dpg.add_button(label="Open Hands", callback=self._open_hands)

            with dpg.table(label="Hand Commands", tag="hand_commands", header_row=True):
                dpg.add_table_column(label="Command", width=40, width_fixed=True)
                for hand_name in hand_names:
                    dpg.add_table_column(label=hand_name)
                with dpg.table_row():
                    dpg.add_text("Des Op Mode")
                    # self._desired_operation_mode = {hand_name: ComboBox(hand_name, hand_operation_modes, suffix="_desired_op_mode") for hand_name in hand_names}
                    for hand_name in hand_names:
                        dpg.add_combo(items=list(hand_operation_modes), default_value="Position Control",
                                      tag=hand_name + desired_operation_suffix, width=int(140 * monitor_scale))
                with dpg.table_row():
                    dpg.add_text("Des Position")
                    self._hand_desired_positions = {
                        hand_name: FloatSlider(hand_name, suffix="_des_position", width=int(140 * monitor_scale),
                                               use_name_as_label=False) for hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Max Effort")
                    self._max_efforts = {
                        hand_name: FloatSlider(hand_name, suffix="_max_effort", width=int(140 * monitor_scale),
                                               max_value=0.8, initial_value=0.3, use_name_as_label=False) for hand_name
                        in hand_names}
                with dpg.table_row():
                    dpg.add_text("Torque On")
                    self._torque_on = {hand_name: CheckBox(hand_name) for hand_name in hand_names}

            with dpg.table(label="Hand States", tag="hand_states", header_row=True):
                dpg.add_table_column(label="State")
                for hand_name in hand_names:
                    dpg.add_table_column(label=hand_name)
                with dpg.table_row():
                    dpg.add_text("Curr Position")
                    self._measured_hand_positions = {hand_name: ValueDisplay(hand_name, suffix="_position") for
                                                     hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Curr Op Mode")
                    self._current_hand_operation = {hand_name: ValueDisplay(hand_name, suffix="_op_mode") for hand_name
                                                    in hand_names}
                with dpg.table_row():
                    dpg.add_text("Temperature")
                    self._current_temp = {hand_name: ValueDisplay(hand_name, suffix="_temperature", initial_value=0)
                                          for
                                          hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Curr Effort")
                    self._current_effort = {hand_name: ValueDisplay(hand_name, suffix="_effort", initial_value=0) for
                                            hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Error Code")
                    self._error_code = {hand_name: ValueDisplay(hand_name, suffix="_error", initial_value=0) for
                                        hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Realtime Tick")
                    self._realtime_tick = {hand_name: ValueDisplay(hand_name, suffix="_time", initial_value=0) for
                                           hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Is Calibrated")
                    self._is_calibrated = {
                        hand_name: ValueDisplay(hand_name, suffix="_is_calibrated", initial_value=False) for
                        hand_name in hand_names}

    def read_hand_states(self, hand_states: Dict[str, EZGripperState]):
        """
        Read and update the current hand states
        :param hand_states: Dictionary of current hand states
        """
        for hand_name in hand_names:
            self._current_hand_operation[hand_name].set(hand_states[hand_name].operation_mode)
            self._current_temp[hand_name].set(hand_states[hand_name].temperature)
            self._measured_hand_positions[hand_name].set(hand_states[hand_name].current_position)
            self._current_effort[hand_name].set(hand_states[hand_name].current_effort)
            self._error_code[hand_name].set(hand_states[hand_name].error_code)
            self._realtime_tick[hand_name].set(hand_states[hand_name].realtime_tick)
            self._is_calibrated[hand_name].set(hand_states[hand_name].is_calibrated)

    def write_hand_commands(self, hand_commands: Dict[str, EZGripperCommand]):
        """
        Write the current hand commands
        :param hand_commands: Dictionary of hand commands to write to
        """
        for hand_name in hand_names:
            hand_commands[hand_name].operation_mode = hand_operation_modes[
                dpg.get_value(item=hand_name + desired_operation_suffix)]
            hand_commands[hand_name].goal_position = self._hand_desired_positions[hand_name].value
            hand_commands[hand_name].max_effort = self._max_efforts[hand_name].value
            hand_commands[hand_name].torque_on = self._torque_on[hand_name].value

    def update_hands(self, data: Dict[str, Any]):
        """
        Update the current hand states and commands
        :param data: Dictionary of robot data
        """
        hand_states = {hand_names[0]: data["left_hand_state"], hand_names[1]: data["right_hand_state"]}
        hand_commands = {hand_names[0]: data["left_hand_command"], hand_names[1]: data["right_hand_command"]}
        self.read_hand_states(hand_states)
        self.write_hand_commands(hand_commands)

    def set_window_positioning(self, x_pos: int = 0, y_pos: int = 0, right_aligned=True):
        """
        Set the positioning of the window based on new x and y positions.
        :param x_pos: New x position of the window
        :param y_pos: New y positions of the window
        :param right_aligned: If right aligned, use x as right side origin. Else, use x as left side origin
        """
        dim = dpg.get_item_rect_size("hand_control_gui")
        if right_aligned:
            dpg.set_item_pos("hand_control_gui", [x_pos - dim[0], y_pos])
        else:
            dpg.set_item_pos("hand_control_gui", [x_pos, y_pos])

    def calibrate_hands(self):
        """
        Set the hands to calibration mode
        """
        for hand_name in hand_names:
            dpg.set_value(item=hand_name + desired_operation_suffix, value=CALIBRATION)

    def reset_errors(self):
        """
        Set the hands to error reset mode
        """
        for hand_name in hand_names:
            dpg.set_value(item=hand_name + desired_operation_suffix, value=ERROR_RESET)

    def enable_hands(self):
        """
        Set the hands to position control and enable torque
        """
        for hand_name in hand_names:
            dpg.set_value(item=hand_name + desired_operation_suffix, value=POSITION_CONTROL)
            self._torque_on[hand_name].set(True)

    def disable_hands(self):
        """
        Disable torque
        """
        for hand_name in hand_names:
            self._torque_on[hand_name].set(False)

    def _close_hands(self):
        """
        Close the hands by setting desired position to 0.0
        """
        for hand_name in hand_names:
            self._hand_desired_positions[hand_name].set(0.0)

    def _open_hands(self):
        """
        Open the hands by setting desired position to 1.0
        """
        for hand_name in hand_names:
            self._hand_desired_positions[hand_name].set(1.0)


if __name__ == '__main__':
    dpg.create_context()
    dpg.configure_app(docking=True, docking_space=True)
    print('a')
    hand_control = HandControlGUI()
    print('b')

    dpg.create_viewport(title='Custom Title', width=500, height=700)
    dpg.setup_dearpygui()
    print('c')
    dpg.show_viewport()
    dpg.start_dearpygui()
