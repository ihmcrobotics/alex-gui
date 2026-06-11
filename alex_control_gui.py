import time
from tkinter.ttk import Combobox
from typing import List, Dict, Literal, Union, Any

import tkinter as tk
from tkinter import *
from tkinter import ttk
from threading import Lock
from copy import deepcopy
import math

import numpy as np
from rich_click.decorators import command
from skrobot.model import RobotModel

from messages import OneDOFJointCommand, OneDOFJointState, AlexCommand, AlexState

DO_NOTHING = "Do Nothing"
HOLD_POSITION = "Hold Position"
USER_CONTROL = "User Control"
robot_control_state = {DO_NOTHING: 0,
                       HOLD_POSITION: 1,
                       USER_CONTROL: 2}


def _initialize_joint_position_sliders(joint_frame: LabelFrame, joint_dict: Dict[str, DoubleVar],
                                       joint_names: List[str], type_: Literal["Position", "Other"] = 'Position',
                                       lower_limits: List[float] | None = None,
                                       upper_limits: List[float] | None = None) -> None:
    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        ttk.Label(joint_frame, text=joint_name).grid(row=i*2 + 1, column=0, sticky="E")
        if type_ == "Position":
            slider = Scale(joint_frame, length=200, orient='horizontal', from_=lower_limits[i], to=upper_limits[i],
                           resolution=0.0001, variable=joint_dict[joint_name])


            slider.grid(row=i*2, column=1, rowspan=2)
        else:
            entry = ttk.Entry(joint_frame, textvariable=joint_dict[joint_name])
            entry.grid(row=i, column=1)


def _initialize_joint_parameter_tabs(notebook: ttk.Notebook, joint_names: List[str],
                                     joint_max_torque: List[float]) -> Dict[str, Dict]:
    joint_max_torques = {}
    joint_stiffness = {}
    joint_damping = {}
    joint_max_pos_error = {}
    joint_max_vel_error = {}
    for i in range(len(joint_names)):
        name = joint_names[i]
        max_torque = joint_max_torque[i]
        joint_max_torques[name] = DoubleVar(value=max_torque)
        joint_stiffness[name] = DoubleVar(value=max_torque/math.pi)
        joint_damping[name] = DoubleVar(value=max_torque/(math.pi*10.0))
        joint_max_pos_error[name] = DoubleVar(value=math.pi)
        joint_max_vel_error[name] = DoubleVar(value=math.pi/10.0)

    impedance_tab = Frame(notebook)
    # impedance_tab.grid(row=0, column=0)
    ttk.Label(impedance_tab, text="Joint").grid(row=0, column=0)
    ttk.Label(impedance_tab, text="Stiffness").grid(row=0, column=1)
    ttk.Label(impedance_tab, text="Damping").grid(row=0, column=2)

    # stiffness_frame = ttk.LabelFrame(impedance_tab, text="Stiffness")
    # stiffness_frame.grid(row=0, column=1)
    # damping_frame = ttk.LabelFrame(impedance_tab, text="Damping")
    # damping_frame.grid(row=0, column=2)

    limit_tab = Frame(notebook)
    ttk.Label(limit_tab, text="Joint").grid(row=0, column=0)
    ttk.Label(limit_tab, text="Max Torque").grid(row=0, column=1)
    ttk.Label(limit_tab, text="Max Pos Error").grid(row=0, column=2)
    ttk.Label(limit_tab, text="Max Vel Error").grid(row=0, column=3)
    joint_count = 1

    horiz_pad = 5
    vert_pad = 2
    for joint_name in joint_names:
        ttk.Label(impedance_tab, text=joint_name).grid(row=joint_count, column=0, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(impedance_tab, textvariable=joint_stiffness[joint_name], ).grid(row=joint_count, column=1, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(impedance_tab, textvariable=joint_damping[joint_name]).grid(row=joint_count, column=2, padx=horiz_pad, pady=vert_pad)

        ttk.Label(limit_tab, text=joint_name).grid(row=joint_count, column=0, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(limit_tab, textvariable=joint_max_torques[joint_name]).grid(row=joint_count, column=1, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(limit_tab, textvariable=joint_max_pos_error[joint_name]).grid(row=joint_count, column=2, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(limit_tab, textvariable=joint_max_vel_error[joint_name]).grid(row=joint_count, column=3, padx=horiz_pad, pady=vert_pad)
        joint_count += 1

    notebook.add(impedance_tab, text="Impedance")
    notebook.add(limit_tab, text="Limits")

    parameter_dict = {"stiffness": joint_stiffness,
                      "damping": joint_damping,
                      "max_torque": joint_max_torques,
                      "max_pos_error": joint_max_pos_error,
                      "max_vel_error": joint_max_vel_error}
    return parameter_dict





class AlexControlGUI:
    def __init__(self, robot: RobotModel): #joint_names: List[str], joint_lower_limits: List[float], joint_upper_limits: List[float], joint_max_torque: List[float]): #, joint_ids: Dict[str, int], joint_lower_limits: np.ndarray, joint_upper_limits: np.ndarray, robot_id: int, ghost_id: int):
        self.control_panel = Tk()
        self.control_panel.title('Alex Control')
        self.content = Frame(self.control_panel)

        self.content.grid(row=0, column=0)
        self._initialize_startup_shutdown_buttons()
        self._initialize_state_buttons()
        self._initialize_control_buttons()
        self.joint_names = robot.joint_names
        max_torques = [joint.max_joint_torque for joint in robot.joint_list]

        self.joint_position_frame = LabelFrame(self.content, text="Joint Position")
        self._joint_positions = {joint: DoubleVar() for joint in self.joint_names}
        self.joint_position_frame.grid(row=1, column=1, rowspan=4, sticky="n")
        _initialize_joint_position_sliders(self.joint_position_frame, self._joint_positions, robot.joint_names,
                                           lower_limits=robot.joint_min_angles, upper_limits=robot.joint_max_angles)

        parameter_notebook = ttk.Notebook(self.content)
        parameter_notebook.grid(row=1, column=2, rowspan=15, sticky="n")
        self._max_torques = dict(zip(self.joint_names, max_torques))

        self._joint_parameter_dict = _initialize_joint_parameter_tabs(parameter_notebook, self.joint_names, max_torques)

        self._begin_time = time.perf_counter_ns()
        self.window_active = True
        self.control_panel.bind("<Destroy>", self.is_destroyed)
        self._reset = False

        joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_list]
        self.alex_command = AlexCommand(joint_commands=joint_commands)

    def _initialize_startup_shutdown_buttons(self):
        main_operation_frame = LabelFrame(self.content, text="Auto Startup/Shutdown", borderwidth=5, relief="ridge",
                                               width=200, height=200)
        main_operation_frame.grid(row=0, column=0, sticky="n")
        secondary_operation_frame = LabelFrame(self.content, text="Manual Startup/Shutdown", borderwidth=5, relief="ridge",
                                          width=200, height=200)
        secondary_operation_frame.grid(row=1, column=0, sticky="n")

        self._request_auto_startup = BooleanVar(value=False)
        self._request_auto_shutdown = BooleanVar(value=False)
        self._request_safe_startup = BooleanVar(value=False)
        self._request_safe_shutdown = BooleanVar(value=False)
        self._enable_actuators = BooleanVar(value=False)
        self._clear_faults = BooleanVar(value=False)
        self._calibrate = BooleanVar(value=False)
        self._servo_robot = BooleanVar(value=False)
        self._unservo_quickly = BooleanVar(value=False)
        self._use_requested_master_gain = BooleanVar(value=True)
        self._master_gain = DoubleVar(value=0.0)
        self._servo_timer = 0.0
        self._servo_initial_value = 0.0
        self._robot_control_state = StringVar(value="Do Nothing")

        self._auto_startup_shutdown_button = Button(main_operation_frame, text="Request Auto Startup", command=self._run_auto_startup_shutdown)
        self._auto_startup_shutdown_button.grid(row=0, column=0, sticky="w")
        Button(main_operation_frame, text="Unservo Quickly", bg="red",
               command=lambda: self._unservo_quickly.set(True)).grid(row=1, column=0, sticky="w")
        self._safe_power_up_down_button = Button(secondary_operation_frame, text="Request Safe Power Up", command=self._run_auto_startup_shutdown)
        self._safe_power_up_down_button.grid(row=0, column=0, sticky="w", columnspan=2)
        ttk.Checkbutton(secondary_operation_frame, text="Enable Actuators", variable=self._enable_actuators).grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(secondary_operation_frame, text="Servo Robot", variable=self._servo_robot).grid(row=3, column=0, sticky="w")
        Button(secondary_operation_frame, text="Clear Faults", command=lambda: self._clear_faults.set(True)).grid(row=1, column=0, sticky="w")
        self._master_gain_display = Label(secondary_operation_frame, text="Master Gain: 0.0")
        self._master_gain_display.grid(row=4, column=0, sticky="w")
        Scale(secondary_operation_frame, variable=self._master_gain, orient='horizontal', from_=0.0, to=1.0,
              resolution=0.01, showvalue=False,
              command=lambda value: self._master_gain_display.config(text="Master Gain: " + str(value))).grid(row=4,
                                                                                                              column=1,
                                                                                                              sticky="w")
        Label(secondary_operation_frame, text="Control State: ").grid(row=5, column=0, sticky="w")
        Combobox(secondary_operation_frame, values=(DO_NOTHING, USER_CONTROL), textvariable=self._robot_control_state).grid(row=5, column=1, sticky="w")

    def _initialize_control_buttons(self):
        control_button_frame = LabelFrame(self.content, text="Control")
        control_button_frame.grid(row=0, column=1, sticky="n")
        self._use_custom_impedance = BooleanVar(value=False)
        self._send_desireds = BooleanVar(value=False)
        self._send_desireds_continuously = BooleanVar(value=False)
        self._reset_joint_positions = BooleanVar(value=True)


        ttk.Checkbutton(control_button_frame, text="Use Custom Impedance", variable=self._use_custom_impedance).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(control_button_frame, text="Send Desireds Continuously", variable=self._send_desireds_continuously).grid(row=0, column=1, sticky="w")
        Button(control_button_frame, text="Send Joint Desireds", command= lambda: self._send_desireds.set(True)).grid(row=0, column=0, sticky="w")
        Button(control_button_frame, text = "Reset Joint Desireds", command= lambda: self._reset_joint_positions.set(True)).grid(row=1, column=0, sticky="w")


    def _on_master_gain_change(self, value):
        self._master_gain_display.config(text=f"{self._:.2f}")

    def _initialize_state_buttons(self):
        self.robot_state_frame = LabelFrame(self.content, text="Robot State", borderwidth=5, relief="ridge",
                                            width=200, height=200)
        self.robot_state_frame.grid(row=2, column=0, sticky="n")

        self._time = DoubleVar(value=0.0)
        self._is_faulted = BooleanVar(value=False)
        self._is_servoing = BooleanVar(value=False)
        self._is_unservoing = BooleanVar(value=False)
        self._is_servoed = BooleanVar(value=False)
        self._are_actuators_enabled = BooleanVar(value=False)
        self._safe_power_up_complete = BooleanVar(value=False)
        self._safe_power_down_complete = BooleanVar(value=False)
        self._current_ll_master_gain = DoubleVar(value=0.0)
        self._auto_shutdown_complete = BooleanVar(value=True)
        self._auto_startup_complete = BooleanVar(value=False)

        ttk.Label(self.robot_state_frame, text="Time: ").grid(row=0, column=0, sticky="w")
        ttk.Label(self.robot_state_frame, textvariable=self._time).grid(row=0, column=1, sticky="w")
        ttk.Label(self.robot_state_frame, text="LL Master Gain: ").grid(row=1, column=0, sticky="w")
        ttk.Label(self.robot_state_frame, textvariable=self._current_ll_master_gain).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Robot Faulted",
                        variable=self._is_faulted).grid(row=2, column=0, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Servoing Robot",
                        variable=self._is_servoing).grid(row=3, column=0, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Unservoing Robot",
                        variable=self._is_unservoing).grid(row=3, column=1, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Robot Servoed",
                        variable=self._is_servoed).grid(row=4, column=0, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Actuators Enabled",
                        variable=self._are_actuators_enabled).grid(row=2, column=1, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Safe Power Up Complete",
                        variable=self._safe_power_up_complete).grid(row=5, column=0, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Safe Power Down Complete",
                        variable=self._safe_power_down_complete).grid(row=5, column=1, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Auto Shutdown Complete",
                        variable=self._auto_shutdown_complete).grid(row=6, column=1, sticky="w")
        ttk.Checkbutton(self.robot_state_frame, text="Auto Startup Complete",
                        variable=self._auto_startup_complete).grid(row=6, column=0, sticky="w")



    def update_gui(self):
        self._update_auto_startup_shutdown()
        self._update_safe_power_up_down()
        self._run_servo()
        self.control_panel.update()


    def _reset_sliders(self, joint_states: List[OneDOFJointState]):
        for state in joint_states:
            name = state.joint_name
            if name in self.joint_names:
                self._joint_positions[name].set(state.q)
        self._reset_joint_positions.set(False)
        self._send_desireds.set(True)

    def run_gui(self, lock: Union[Lock, None] = None, shared_data: Union[Dict[str, Any], None] = None):
        while self.window_active:
            if lock is not None and shared_data is not None:
                with lock:
                    self._read_state(shared_data["alex_state"])
                    self._write_command(shared_data["alex_command"])
            self.update_gui()
            time.sleep(0.01)

    def _read_state(self, alex_state: AlexState):
        self._time.set(alex_state.time)
        self._is_faulted.set(alex_state.is_faulted)
        self._is_servoing.set(alex_state.is_servoing)
        self._is_unservoing.set(alex_state.is_unservoing)
        self._is_servoed.set(alex_state.is_servoed)
        self._are_actuators_enabled.set(alex_state.are_actuators_enabled)
        self._safe_power_up_complete.set(alex_state.safe_power_up_complete)
        self._safe_power_down_complete.set(alex_state.safe_power_down_complete)
        self._current_ll_master_gain.set(alex_state.current_low_level_master_gain)
        self._auto_shutdown_complete.set(alex_state.auto_shutdown_complete)
        self._auto_startup_complete.set(alex_state.auto_startup_complete)

        if self._reset_joint_positions.get():
            self._reset_sliders(alex_state.joint_states)

    def _write_command(self, alex_command: AlexCommand):
        alex_command.request_auto_startup = self._request_auto_startup.get()
        alex_command.request_auto_shutdown = self._request_auto_shutdown.get()
        alex_command.number_of_joints = len(self.joint_names)
        alex_command.clear_faults = self._clear_faults.get()
        self._clear_faults.set(False)
        alex_command.request_enable_actuators = self._enable_actuators.get()
        alex_command.request_disable_actuators = not self._enable_actuators.get()
        alex_command.requested_master_gain = self._master_gain.get()
        alex_command.use_requested_master_gain = True
        alex_command.robot_control_state = robot_control_state[self._robot_control_state.get()]

        if self._send_desireds.get() or self._send_desireds_continuously.get():
            self._update_desireds(alex_command.joint_commands)
            self._send_desireds.set(False)

    def _update_desireds(self, commands: List[OneDOFJointCommand]):
        for command in commands:
            name = command.joint_name
            command.q_des = self._joint_positions[name].get()
            command.qd_des = 0.0
            command.taw_des = 0.0
            if self._use_custom_impedance.get():
                command.stiffness = self._joint_parameter_dict["stiffness"][name].get()
                command.damping = self._joint_parameter_dict["damping"][name].get()
                command.max_torque = self._joint_parameter_dict["max_torque"][name].get()
                command.max_position_error = self._joint_parameter_dict["max_pos_error"][name].get()
                command.max_velocity_error = self._joint_parameter_dict["max_vel_error"][name].get()
            else:
                command.stiffness = math.nan
                command.damping = math.nan
                command.max_torque = math.nan
                command.max_position_error = math.nan
                command.max_velocity_error = math.nan

    def _check_limits(self, joint_name: str):
        requested_max_torque = self._joint_parameter_dict["max_torque"][joint_name]
        max_position_error = self._joint_parameter_dict["max_pos_error"][joint_name]
        max_velocity_error = self._joint_parameter_dict["max_vel_error"][joint_name]
        stiffness = self._joint_parameter_dict["stiffness"][joint_name].get()
        damping = self._joint_parameter_dict["damping"][joint_name].get()

        if self._max_torques[joint_name] < requested_max_torque.get():
            print (joint_name + " max torque exceeded")
            max_torque = self._max_torques[joint_name]
            requested_max_torque.set(max_torque)
        else:
            max_torque = requested_max_torque.get()

        if stiffness > 0.0:
            max_position_error.set(min(max_position_error.get(), max_torque / stiffness))

        if damping > 0.0:
            max_velocity_error.set(min(max_velocity_error.get(), max_torque / damping))

    def _run_servo(self):
        if (self._servo_robot.get() and self._master_gain.get() == 0.0) or self._master_gain.get() > 0.0:
            self._servo_timer = time.time()
            self._servo_initial_value = self._current_ll_master_gain.get()
        servo_time_elapsed = time.time() - self._servo_timer

        servo_ratio = servo_time_elapsed / 2.0
        if servo_ratio <= 1.0:
            self._master_gain.set(servo_ratio if self._servo_robot.get() else self._servo_initial_value * (1.0 - servo_ratio))


    def _run_auto_startup_shutdown(self):
        print(self._request_auto_startup.get(), self._request_auto_shutdown.get())
        print(self._auto_startup_complete.get(), self._auto_shutdown_complete.get())
        if not self._request_auto_startup.get() and not self._request_auto_shutdown.get():
            if not self._auto_shutdown_complete.get() and not self._auto_startup_complete.get():
                self._auto_startup_shutdown_button.config(text="Starting up, press to stop")
                self._request_auto_startup.set(True)
                self._request_auto_shutdown.set(False)
                print("Requesting Auto Startup")
            elif self._auto_startup_complete.get():
                self._auto_startup_shutdown_button.config(text="Shutting down", state="disabled")
                self._request_auto_shutdown.set(True)
                self._auto_startup_complete.set(False)
                print("Shutting down")
            elif self._auto_shutdown_complete.get():
                self._auto_startup_shutdown_button.config(text="Starting up, press to stop")
                self._request_auto_startup.set(True)
                self._auto_shutdown_complete.set(False)
                print("Starting up")
        elif self._request_auto_startup.get():
            self._auto_startup_shutdown_button.config(text="Shutting down", state="disabled")
            self._request_auto_shutdown.set(True)
            self._request_auto_startup.set(False)
            print("Shutting down midway through startup")
        if self._request_auto_startup.get():
            self._reset_joint_positions.set(True)
        self._begin_time = time.perf_counter_ns()

    def _update_auto_startup_shutdown(self):
        if self._request_auto_startup.get() and self._auto_startup_complete.get():
            self._auto_startup_shutdown_button.config(text="Request Auto Shutdown", state="normal")
            self._request_auto_startup.set(False)
            self._enable_actuators.set(True)
            self._servo_robot.set(True)
            self._robot_control_state.set("User Control")
            print("auto startup complete")
        elif self._request_auto_shutdown.get() and self._auto_shutdown_complete.get():
            self._auto_startup_shutdown_button.config(text="Request Auto Startup", state="normal")
            self._request_auto_shutdown.set(False)
            self._enable_actuators.set(False)
            self._servo_robot.set(False)
            self._robot_control_state.set("Do Nothing")
            print("auto shutdown complete")

    def _run_safe_power_up_down(self):
        if not self._request_safe_startup.get() and not self._request_safe_shutdown.get():
            if not self._safe_power_down_complete.get() and not self._safe_power_up_complete.get():
                self._safe_power_up_down_button.config(text="Starting up, press to stop")
                self._request_safe_startup.set(True)
                self._request_safe_shutdown.set(False)
            elif self._safe_power_up_complete.get():
                self._safe_power_up_down_button.config(text="Shutting down", state="disabled")
                self._request_safe_shutdown.set(True)
                self._safe_power_up_complete.set(False)
                print("Shutting down")
            elif self._safe_power_down_complete:
                self._safe_power_up_down_button.config(text="Starting up, press to stop")
                self._request_safe_startup.set(True)
                self._safe_power_down_complete.set(False)
                print("Starting up")
            elif self._request_safe_startup.get():
                self._safe_power_up_down_button.config(text="Shutting down", state="disabled")
                self._request_safe_shutdown.set(True)
                self._request_safe_startup.set(False)
                print("Shutting down midway through startup")
        self._begin_time = time.perf_counter_ns()

    def _update_safe_power_up_down(self):
        if self._request_safe_startup.get() and (self._safe_power_up_complete.get() or self.time_elapsed() > 1.0):
            self._safe_power_up_down_button.config(text="Request safe Shutdown", state="normal")
            self._request_safe_startup.set(False)
            print("safe startup complete")
        elif self._request_safe_shutdown.get() and (self._safe_power_down_complete.get() or self.time_elapsed() > 1.0):
            self._safe_power_up_down_button.config(text="Request safe Startup", state="normal")
            self._request_safe_shutdown.set(False)
            print("safe shutdown complete")

    def time_elapsed(self):
        return (time.perf_counter_ns() - self._begin_time) * 1.0e-9


    def is_destroyed(self, event):
        if event.widget != self.control_panel:
            return
        self.window_active = False
        print("Window closed, shutting down")






import threading
if __name__ == "__main__":
    path = "/ihmc-alex-sdk/alex-models/alex_description/urdf/002/hehehe.urdf"
    robot = RobotModel.from_urdf(path)
    gui = AlexControlGUI(robot)

    gui.run_gui()




