import time
from typing import List, Dict, Literal

import tkinter as tk
from tkinter import *
from tkinter import ttk
from threading import Lock
from copy import deepcopy

import numpy as np
from rich_click.decorators import command

from messages import OneDOFJointCommand, OneDOFJointState


def _initialize_joint_position_sliders(joint_frame: LabelFrame, joint_dict: Dict[str, DoubleVar],
                                       joint_names: List[str], type_: Literal["Position", "Other"] = 'Position',
                                       lower_limits: List[float] | None = None,
                                       upper_limits: List[float] | None = None) -> None:
    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        label = ttk.Label(joint_frame, text=joint_name)
        label.grid(row=i, column=0)
        if type_ == "Position":
            slider = ttk.Scale(joint_frame, length=200, orient='horizontal', from_=lower_limits[i], to=upper_limits[i],
                       variable=joint_dict[joint_name])


            slider.grid(row=i, column=1)
        else:
            entry = ttk.Entry(joint_frame, textvariable=joint_dict[joint_name])
            entry.grid(row=i, column=1)


def _initialize_joint_parameter_tabs(notebook: ttk.Notebook, joint_names: List[str]):
    joint_stiffness = {joint: DoubleVar() for joint in joint_names}
    joint_damping = {joint: DoubleVar() for joint in joint_names}
    joint_max_torque = {joint: DoubleVar() for joint in joint_names}
    joint_max_pos_error = {joint: DoubleVar() for joint in joint_names}
    joint_max_vel_error = {joint: DoubleVar() for joint in joint_names}

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
        ttk.Entry(limit_tab, textvariable=joint_max_torque[joint_name]).grid(row=joint_count, column=1, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(limit_tab, textvariable=joint_max_pos_error[joint_name]).grid(row=joint_count, column=2, padx=horiz_pad, pady=vert_pad)
        ttk.Entry(limit_tab, textvariable=joint_max_vel_error[joint_name]).grid(row=joint_count, column=3, padx=horiz_pad, pady=vert_pad)
        joint_count += 1

    notebook.add(impedance_tab, text="Impedance")
    notebook.add(limit_tab, text="Limits")

    parameter_dict = {"stiffness": joint_stiffness,
                      "damping": joint_damping,
                      "max_torque": joint_max_torque,
                      "max_pos_error": joint_max_pos_error,
                      "max_vel_error": joint_max_vel_error}
    return parameter_dict





class AlexControlGUI:
    def __init__(self, joint_names: List[str], joint_lower_limits: List[float], joint_upper_limits: List[float]): #, joint_ids: Dict[str, int], joint_lower_limits: np.ndarray, joint_upper_limits: np.ndarray, robot_id: int, ghost_id: int):
        self.control_panel = Tk()
        self.control_panel.title('Alex Control')
        self.content = Frame(self.control_panel)

        self.content.grid(row=0, column=0)
        self._initialize_startup_shutdown_buttons()
        self._initialize_status_buttons()
        self._initialize_control_buttons()
        self.joint_names = joint_names

        if (joint_names is not None):
            self.joint_position_frame = LabelFrame(self.content, text="Joint Position")
            self._joint_positions = {joint: DoubleVar() for joint in joint_names}
            self.joint_position_frame.grid(row=1, column=1, rowspan=4, sticky="n")
            _initialize_joint_position_sliders(self.joint_position_frame, self._joint_positions, joint_names, lower_limits=joint_lower_limits, upper_limits=joint_upper_limits)

            parameter_notebook = ttk.Notebook(self.content)
            parameter_notebook.grid(row=1, column=2, rowspan=15, sticky="n")
            self._joint_parameter_dict = _initialize_joint_parameter_tabs(parameter_notebook, joint_names)

        self._begin_time = time.perf_counter_ns()
        self.window_active = True
        self.control_panel.bind("<Destroy>", self.is_destroyed)
        self._reset = False

    def _initialize_startup_shutdown_buttons(self):
        command_button_frame = LabelFrame(self.content, text="Startup/Shutdown", borderwidth=5, relief="ridge",
                                               width=200, height=200)
        command_button_frame.grid(row=0, column=0, sticky="n")

        self.request_auto_startup = BooleanVar(value=False)
        self.request_auto_shutdown = BooleanVar(value=False)
        self.enable_actuators = BooleanVar(value=False)
        self.servo_robot = BooleanVar(value=False)
        self.unservo_quickly = BooleanVar(value=False)
        self.clear_faults = BooleanVar(value=False)

        Button(command_button_frame, text="Request Auto Startup/Shutdown",command=self._begin_startup_shutdown).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(command_button_frame, text="Enable Actuators", variable=self.enable_actuators).grid(row=1, column=0, sticky="w")
        ttk.Checkbutton(command_button_frame, text="Servo Robot", variable=self.servo_robot).grid(row=2, column=0, sticky="w")
        Button(command_button_frame, text="Unservo Quickly", bg="red", command=lambda: self.unservo_quickly.set(True)).grid(row=0, column=1, sticky="w")
        ttk.Checkbutton(command_button_frame, text="Clear Faults", variable=self.clear_faults).grid(row=1, column=1, sticky="w")

    def _initialize_control_buttons(self):
        control_button_frame = LabelFrame(self.content, text="Control")
        control_button_frame.grid(row=0, column=1, sticky="n")
        self._use_custom_impedance = BooleanVar(value=False)
        self._send_desireds = BooleanVar(value=False)
        self._send_desireds_continuously = BooleanVar(value=False)
        self._reset_joint_positions = BooleanVar(value=False)

        ttk.Checkbutton(control_button_frame, text="Use Custom Impedance", variable=self._use_custom_impedance).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(control_button_frame, text="Send Desireds Continuously", variable=self._send_desireds_continuously).grid(row=0, column=1, sticky="w")
        Button(control_button_frame, text="Send Joint Desireds", command= lambda: self._send_desireds.set(True)).grid(row=0, column=0, sticky="w")
        Button(control_button_frame, text = "Reset Joint Desireds", command= lambda: self._reset_joint_positions.set(True)).grid(row=1, column=0, sticky="w")

    def _initialize_status_buttons(self):
        self.status_button_frame = LabelFrame(self.content, text="Status Buttons", borderwidth=5, relief="ridge",
                                              width=200, height=200)

        self.auto_shutdown_complete = BooleanVar(value=False)
        self.auto_startup_complete = BooleanVar(value=False)

        self.auto_shutdown_complete_button = ttk.Checkbutton(self.status_button_frame, text="Auto-Shutdown Complete",
                                                             variable=self.auto_shutdown_complete)
        self.auto_startup_complete_button = ttk.Checkbutton(self.status_button_frame, text="Auto-Startup Complete",
                                                            variable=self.auto_startup_complete)

        self.status_button_frame.grid(row=1, column=0, sticky="n")
        self.auto_startup_complete_button.grid(row=0, column=0, sticky="w")
        self.auto_shutdown_complete_button.grid(row=1, column=0, sticky="w")

    def update_gui(self):
        self.control_panel.update()
        self.update_startup_shutdown(False)

    def _reset_sliders(self, joint_states: Dict[str, OneDOFJointState]):
        for name in self.joint_names:
            self._joint_positions[name].set(joint_states[name].q)
        self._reset_joint_positions.set(False)

    def run_gui(self, lock: Lock, shared_data: Dict):
        # self.root.mainloop()
        while self.window_active:
            self.update_gui()
            with lock:
                if self._reset_joint_positions.get():
                    self._reset_sliders(shared_data["joint_states"])
                if self._send_desireds.get() or self._send_desireds_continuously.get():
                    self._write_desireds(shared_data["joint_commands"])
                    self._send_desireds.set(False)
            time.sleep(0.01)

    def _write_desireds(self, data: Dict[str, OneDOFJointCommand]):
        for name in self.joint_names:
            joint_desired = data[name]
            joint_desired.q_des = self._joint_positions[name].get()
            joint_desired.qd_des = 0.0
            joint_desired.taw_des = 0.0
            joint_desired.stiffness = self._joint_parameter_dict["stiffness"][name].get()
            joint_desired.damping = self._joint_parameter_dict["damping"][name].get()
            joint_desired.max_torque = self._joint_parameter_dict["max_torque"][name].get()
            joint_desired.max_position_error = self._joint_parameter_dict["max_pos_error"][name].get()
            joint_desired.max_velocity_error = self._joint_parameter_dict["max_vel_error"][name].get()


    def _begin_startup_shutdown(self):
        print("Attempting")
        if not self.request_auto_startup.get() and not self.request_auto_shutdown.get():
            if not self.auto_shutdown_complete.get() and not self.auto_startup_complete.get():
                self.request_auto_startup.set(True)
                self.request_auto_shutdown.set(False)
            elif self.auto_startup_complete.get():
                self.request_auto_shutdown.set(True)
                self.auto_startup_complete.set(False)
                print("Shutting down")
            elif self.auto_shutdown_complete:
                self.request_auto_startup.set(True)
                self.auto_shutdown_complete.set(False)
                print("Starting up")
            elif self.request_auto_startup.get():
                self.request_auto_shutdown.set(True)
                self.request_auto_startup.set(False)
                print("Shutting down midway through startup")
        self._begin_time = time.perf_counter_ns()

    def update_startup_shutdown(self, startup_shutdown_complete):
        if self.request_auto_startup.get() and (startup_shutdown_complete or self.time_elapsed() > 1.0):
            self.request_auto_startup.set(False)
            self.auto_startup_complete.set(True)
            print("auto startup complete")
        elif self.request_auto_shutdown.get() and (startup_shutdown_complete or self.time_elapsed() > 1.0):
            self.request_auto_shutdown.set(False)
            self.auto_shutdown_complete.set(True)
            print("auto shutdown complete")

    def time_elapsed(self):
        return (time.perf_counter_ns() - self._begin_time) * 1.0e-9


    def is_destroyed(self, event):
        if event.widget != self.control_panel:
            return
        self.window_active = False
        print("Window closed, shutting down")






import threading
if __name__ == "__main__":
    joint_list = ["heh", "meh", "leh", "teh"]
    gui = AlexControlGUI(joint_list)

    gui.run_gui()




