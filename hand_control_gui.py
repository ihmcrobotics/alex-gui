import threading
from typing import List, Dict

import dearpygui.dearpygui as dpg
from dearpygui_helpers import *
from messages import OneDOFJointCommand
hand_names = ["left_hand", "right_hand"]


hand_operation_modes = {"Position Control": 0,
                        "Calibration": 1,
                        "Error Reset": 2}

class HandControlGUI:
    def __init__(self):

        with dpg.window(label="Hand Control", tag="hand_control_gui", pos=[0, 550], width=500, height=400):
            self._send_hand_desireds = Button("Send Hand Desireds")
            self._send_hand_desireds_continuously = CheckBox("Send Hand Desireds Continuously")
            with dpg.table(label="Hand Commands", tag="hand_commands", header_row=True):
                dpg.add_table_column(label="Command")
                for hand_name in hand_names:
                    dpg.add_table_column(label=hand_name)
                with dpg.table_row():
                    dpg.add_text("Operation Mode")
                    for hand_name in hand_names:
                        dpg.add_combo(items=list(hand_operation_modes), default_value="Position Control", tag=hand_name+"_desired_op_mode", width=150)
                with dpg.table_row():
                    dpg.add_text("Position")
                    self._hand_desired_positions = {hand_name: FloatSlider(hand_name, suffix="_des_position", width=150, use_name_as_label=False) for hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Max Effort")
                    self._max_efforts = {hand_name: FloatSlider(hand_name, suffix="_max_effort", width=150, max_value=0.8, initial_value=0.3, use_name_as_label=False) for hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Torque On")
                    self._torque_on = {hand_name: CheckBox(hand_name) for hand_name in hand_names}
            dpg.add_text("")
            dpg.add_text("")
            with dpg.table(label="Hand States", tag="hand_states", header_row=True):
                dpg.add_table_column(label="State")
                for hand_name in hand_names:
                    dpg.add_table_column(label=hand_name)
                with dpg.table_row():
                    dpg.add_text("Position")
                    self._measured_hand_positions = {hand_name: ValueDisplay(hand_name, suffix="_position") for hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Operation Mode")
                    self._current_hand_operation = {hand_name: ValueDisplay(hand_name, suffix="_op_mode") for hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Temperature")
                    self._current_temp = {hand_name: ValueDisplay(hand_name, suffix="_temperature", initial_value=0)
                                            for
                                            hand_name in hand_names}
                with dpg.table_row():
                    dpg.add_text("Effort")
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
                    self._is_calibrated = {hand_name: ValueDisplay(hand_name, suffix="_is_calibrated", initial_value=False) for
                                                    hand_name in hand_names}

    def read_hand_states(self, lock: threading.Lock, data: Dict[str, Any]):
        with lock:
            hand_states = {hand_names[0]: data["left_hand_state"], hand_names[1]: data["right_hand_state"]}
            for hand_name in hand_names:
                self._current_hand_operation[hand_name].set(hand_states[hand_name].operation_mode)
                self._current_temp[hand_name].set(hand_states[hand_name].temperature)
                self._measured_hand_positions[hand_name].set(hand_states[hand_name].current_position)
                self._current_effort[hand_name].set(hand_states[hand_name].current_effort)
                self._error_code[hand_name].set(hand_states[hand_name].error_code)
                self._realtime_tick[hand_name].set(hand_states[hand_name].realtime_tick)
                self._is_calibrated[hand_name].set(hand_states[hand_name].is_calibrated)

    # def write_hand_commands(self, lock: threading.Lock, data: Dict[str, Any]):
    #     with lock:
    #         hand_commands = {hand_names[0]: data["left_hand_command"], hand_names[1]: data["right_hand_command"]}
    #         for hand_name in hand_names:
    #             hand_commands[hand_name].operation_mode = self.



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
