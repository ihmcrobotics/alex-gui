import math
from typing import List, Dict

from skrobot.model import RobotModel, Joint
from tkinter import ttk
from tkinter import *

from messages import OneDOFJointState, OneDOFJointCommand


def initialize_joint_position_sliders(joint_frame: LabelFrame, joint_dict: Dict[str, DoubleVar],
                                       joint_names: List[str],
                                       lower_limits: List[float] | None = None,
                                       upper_limits: List[float] | None = None) -> None:
    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        ttk.Label(joint_frame, text=joint_name).grid(row=i*2 + 1, column=0, sticky="E")
        slider = Scale(joint_frame, length=200, orient='horizontal', from_=lower_limits[i], to=upper_limits[i],
                       resolution=0.0001, variable=joint_dict[joint_name])


        slider.grid(row=i*2, column=1, rowspan=2)


def initialize_joint_parameter_tabs(notebook: ttk.Notebook, joint_names: List[str],
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
        joint_stiffness[name] = DoubleVar(value=max_torque/math.pi*2.0)
        joint_damping[name] = DoubleVar(value=max_torque/(math.pi*5.0))
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

class ArmControlGUI:
    def __init__(self, frame, robot: RobotModel):
        self._frame = frame
        self.robot = robot

        self.joint_names = []
        self.joint_list = []
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        self.joint_max_torques = []
        self._max_torque_dict = {}
        for joint in robot.joint_list:
            if "ezgripper" not in joint.name:
                self.joint_names.append(joint.name)
                self.joint_list.append(joint)
                self.joint_lower_limits.append(joint.min_joint_angle)
                self.joint_upper_limits.append(joint.max_joint_angle)
                self.joint_max_torques.append(joint.max_joint_torque)
                self._max_torque_dict[joint.name] = joint.max_joint_torque

        self.joint_position_frame = LabelFrame(self._frame, text="Joint Position")
        self.desired_joint_positions = {joint: DoubleVar() for joint in self.joint_names}
        self.measured_joint_positions = {joint: 0.0 for joint in self.joint_names}
        self.joint_position_frame.grid(row=0, column=1, rowspan=2, sticky="n")

        initialize_joint_position_sliders(self.joint_position_frame, self.desired_joint_positions, self.joint_names,
                                          lower_limits=self.joint_lower_limits, upper_limits=self.joint_upper_limits)

        parameter_notebook = ttk.Notebook(self._frame)
        parameter_notebook.grid(row=0, column=2, rowspan=3, sticky="n")

        self._joint_parameter_dict = initialize_joint_parameter_tabs(parameter_notebook, self.joint_names, self.joint_max_torques)

    def reset_sliders(self):
        for name in self.joint_names:
            self.desired_joint_positions[name].set(self.measured_joint_positions[name])

    def updated_measured(self, joint_states: List[OneDOFJointState]):
        for joint_state in joint_states:
            name = joint_state.joint_name
            if name in self.joint_names:
                self.measured_joint_positions[name] = joint_state.q

    def update_desireds(self, commands: List[OneDOFJointCommand], use_custom_impedance: bool):
        for command in commands:
            name = command.joint_name
            if name in self.joint_names:
                command.q_des = self.desired_joint_positions[name].get()
                command.qd_des = 0.0
                command.taw_des = 0.0
                if use_custom_impedance:
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

        if self._max_torque_dict[joint_name] < requested_max_torque.get():
            print (joint_name + " max torque exceeded")
            max_torque = self._max_torque_dict[joint_name]
            requested_max_torque.set(max_torque)
        else:
            max_torque = requested_max_torque.get()

        if stiffness > 0.0:
            max_position_error.set(min(max_position_error.get(), max_torque / stiffness))

        if damping > 0.0:
            max_velocity_error.set(min(max_velocity_error.get(), max_torque / damping))


