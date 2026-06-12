import time
from tkinter.ttk import Combobox
from typing import List, Dict, Literal, Union, Any
from poses import *
from tkinter import *
from tkinter import ttk
from threading import Lock
import math
from skrobot.model import RobotModel
from alex_data_gui import *
from arm_control_gui import *

from messages import OneDOFJointCommand, OneDOFJointState, AlexCommand, AlexState

DO_NOTHING = "Do Nothing"
HOLD_POSITION = "Hold Position"
USER_CONTROL = "User Control"
robot_control_state = {DO_NOTHING: 0,
                       HOLD_POSITION: 1,
                       USER_CONTROL: 2}





class AlexControlGUI:
    def __init__(self, robot: RobotModel): #joint_names: List[str], joint_lower_limits: List[float], joint_upper_limits: List[float], joint_max_torque: List[float]): #, joint_ids: Dict[str, int], joint_lower_limits: np.ndarray, joint_upper_limits: np.ndarray, robot_id: int, ghost_id: int):
        self.control_panel = Tk()
        self.control_panel.title('Alex Control')
        self.content = Frame(self.control_panel)

        self.content.grid(row=0, column=0)
        self._initialize_startup_shutdown_buttons()
        self._initialize_state_buttons()
        self._initialize_control_buttons()

        self._arm_control = ArmControlGUI(self.control_panel, robot)
        self.plotter = AlexDataGUI(self.control_panel, self._arm_control)
        self._initialize_pose_buttons()

        self._begin_time = time.perf_counter_ns()
        self.window_active = True
        self.control_panel.bind("<Destroy>", self.is_destroyed)
        self._reset = False

        joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_list]
        self.alex_command = AlexCommand(joint_commands=joint_commands, number_of_joints=len(robot.joint_names))


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
        self._high_level_servo_complete = True
        self._unservo_quickly = BooleanVar(value=False)
        self._use_requested_master_gain = BooleanVar(value=True)
        self._requested_master_gain = DoubleVar(value=0.0)
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
        self._servo_robot_button = Button(secondary_operation_frame, text="Servo Robot", command=self._start_servo_unservo)
        self._servo_robot_button.grid(row=3, column=0, sticky="w")
        Button(secondary_operation_frame, text="Clear Faults", command=lambda: self._clear_faults.set(True)).grid(row=1, column=0, sticky="w")
        self._master_gain_display = Label(secondary_operation_frame, text="Requested Master Gain: 0.0")
        self._master_gain_display.grid(row=4, column=0, sticky="w")
        self._master_gain_scale = Scale(secondary_operation_frame, variable=self._requested_master_gain,
                                        orient='horizontal', from_=0.0, to=1.0, resolution=0.01, showvalue=False)
        self._master_gain_scale.grid(row=4, column=1, sticky="w")
        Label(secondary_operation_frame, text="Control State: ").grid(row=5, column=0, sticky="w")
        Combobox(secondary_operation_frame, values=(DO_NOTHING, USER_CONTROL), textvariable=self._robot_control_state).grid(row=5, column=1, sticky="w")

    def _initialize_control_buttons(self):
        control_button_frame = LabelFrame(self.content, text="Control")
        control_button_frame.grid(row=2, column=0, sticky="n")
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
        self.robot_state_frame.grid(row=3, column=0, sticky="n")

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

    def _initialize_pose_buttons(self):
        pose_frame = LabelFrame(self.content, text="Poses", borderwidth=5, relief="ridge")
        pose_frame.grid(row=4, column=0, sticky="n")
        self._home_pose = BooleanVar(value=False)
        self._arms_up_pose = BooleanVar(value=False)
        self._pose_running = False
        self._pose_start_time = time.time()
        self._pose_duration = 10.0
        self._pose_joints = []
        joint_positions = self._arm_control.desired_joint_positions
        joint_names = self._arm_control.joint_names
        self._initial_positions = [joint_positions[name].get() for name in joint_names]
        self._final_positions = [joint_positions[name].get() for name in joint_names]
        self._curr_positions = [joint_positions[name].get() for name in joint_names]

        Button(pose_frame, text="Home", command=lambda: self._home_pose.set(True)).grid(row=0, column=0, sticky="w")
        Button(pose_frame, text="Arms Up", command=lambda: self._arms_up_pose.set(True)).grid(row=0, column=1, sticky="w")

    def _run_poses(self):
        if self._home_pose.get() or self._arms_up_pose.get():
            self._send_desireds_continuously.set(False)
            if not self._pose_running:
                self._pose_start_time = time.time()
                self._pose_running = True
                if self._home_pose.get():
                    self._pose_joints = home_pose_joints
                    self._final_positions = home_pose_values
                elif self._arms_up_pose.get():
                    self._pose_joints = arms_up_pose_joints
                    self._final_positions = arms_up_pose_values
                self._initial_positions = [self._arm_control.desired_joint_positions[name].get() for name in self._pose_joints]
                print("Starting move to home pose")
            else:
                elapsed_time = time.time() - self._pose_start_time
                if elapsed_time < self._pose_duration:
                    update_pose_command(elapsed_time, self._pose_duration, self._initial_positions, self._final_positions, self._curr_positions)
                    for i in range(len(self._final_positions)):
                        self._arm_control.desired_joint_positions[self._pose_joints[i]].set(self._curr_positions[i])
                else:
                    for i in range(len(self._final_positions)):
                        self._arm_control.desired_joint_positions[self._pose_joints[i]].set(self._final_positions[i])
                    self._pose_running = False
                    self._home_pose.set(False)
                    self._arms_up_pose.set(False)
                    print("Completed move to home pose")
                self._send_desireds.set(True)


    def update_gui(self):
        self._update_auto_startup_shutdown()
        self._update_safe_power_up_down()
        if not self._high_level_servo_complete:
            self._run_servo_unservo()
        self._run_poses()
        self.control_panel.update()


    def _reset_sliders(self, joint_states: List[OneDOFJointState]):
        self._arm_control.reset_sliders(joint_states)
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

        self._arm_control.updated_measured(alex_state.joint_states)
        if self._reset_joint_positions.get():
            self._arm_control.reset_sliders()
            self._reset_joint_positions.set(False)
            self._send_desireds.set(True)

    def _write_command(self, alex_command: AlexCommand):
        alex_command.request_auto_startup = self._request_auto_startup.get()
        alex_command.request_auto_shutdown = self._request_auto_shutdown.get()
        alex_command.clear_faults = self._clear_faults.get()
        self._clear_faults.set(False)
        alex_command.request_enable_actuators = self._enable_actuators.get()
        alex_command.request_disable_actuators = not self._enable_actuators.get()
        alex_command.requested_master_gain = self._requested_master_gain.get()
        alex_command.use_requested_master_gain = True
        alex_command.robot_control_state = robot_control_state[self._robot_control_state.get()]

        if self._send_desireds.get() or self._send_desireds_continuously.get():
            self._arm_control.update_desireds(alex_command.joint_commands, self._use_custom_impedance.get())
            self._send_desireds.set(False)

    def _start_servo_unservo(self):
        if self._servo_robot.get():
            self._servo_robot_button.config(text="Unservoing Robot", state="disabled")
            self._servo_robot.set(False)
            self._high_level_servo_complete = False
            self._servo_timer = time.time()
            self._servo_initial_value = self._current_ll_master_gain.get()
        elif self._high_level_servo_complete:
            self._servo_robot_button.config(text="Servoing Robot, press to cancel", state="normal")
            self._servo_initial_value = 0.0
            self._servo_timer = time.time()
            self._high_level_servo_complete = False


    def _run_servo_unservo(self):
        servo_ratio = (time.time() - self._servo_timer) / 2.0
        print(servo_ratio)
        if servo_ratio < 1.0:
            if self._servo_robot.get():
                # self._master_gain_scale.set(servo_ratio)
                self._requested_master_gain.set(servo_ratio)
            else:
                # self._master_gain_scale.set(self._servo_initial_value* (1.0 - servo_ratio))
                self._requested_master_gain.set(self._servo_initial_value * (1.0 - servo_ratio))
            print(self._requested_master_gain.get())
        else:
            self._high_level_servo_complete = True
            if self._servo_robot.get():
                self._servo_robot_button.config(text="Unservo Robot", state="normal")
                self._requested_master_gain.set(1.0)
            else:
                self._servo_robot_button.config(text="Servo Robot", state="normal")
                self._requested_master_gain.set(0.0)



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
            self._start_servo_unservo()
            self._robot_control_state.set("User Control")
            print("auto startup complete")
        elif self._request_auto_shutdown.get() and self._auto_shutdown_complete.get():
            self._auto_startup_shutdown_button.config(text="Request Auto Startup", state="normal")
            self._request_auto_shutdown.set(False)
            self._enable_actuators.set(False)
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




