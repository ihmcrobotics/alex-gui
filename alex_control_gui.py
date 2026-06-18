import time
from typing import List, Dict, Literal, Union, Any

from poses import *
from threading import Lock
import math
from skrobot.model import RobotModel
from arm_control_gui import *
import dearpygui.dearpygui as dpg
from dearpygui_helpers import *

from messages import OneDOFJointCommand, OneDOFJointState, AlexCommand, AlexState

DO_NOTHING = "Do Nothing"
HOLD_POSITION = "Hold Position"
USER_CONTROL = "User Control"
robot_control_state = {DO_NOTHING: 0,
                       HOLD_POSITION: 1,
                       USER_CONTROL: 2}

class AlexControlGUI:
    def __init__(self, robot: RobotModel, frequency: float= 100.0):
        self.frequency = frequency
        self.dt = 1.0 / self.frequency

        print("ah")
        self._initialize_startup_shutdown_buttons()
        print("bah")
        self._initialize_state_buttons()
        print("cah")
        self._arm_control = ArmControlGUI(robot)
        self._hand_control = HandControlGUI()
        self._initialize_pose_buttons()
        self._reset = False

        self._avg_loop_time = 0.0
        self._loop_start_time = time.perf_counter_ns()
        self._loop_num = 0
        self._loops_to_avg = 5 * self.frequency
        self._missed_loops = 0

        joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_list]
        self.alex_command = AlexCommand(joint_commands=joint_commands, number_of_joints=len(robot.joint_names))


    def _initialize_startup_shutdown_buttons(self):

        self._request_auto_startup = False
        self._request_auto_shutdown = False
        self._request_safe_startup = False
        self._request_safe_shutdown = False
        self._calibrate = False
        self._servo_robot = False
        self._high_level_servo_complete = True
        self._unservo_quickly = False
        self._use_requested_master_gain = True
        self._servo_timer = 0.0
        self._servo_initial_value = 0.0
        self._robot_control_state = DO_NOTHING
        self._auto_startup_shutdown_tag = "request_auto"
        self._safe_power_up_down_tag = "request_safe"
        print("heh")

        with dpg.window(label="Auto Startup/Shutdown", tag="auto_startup_shutdown", pos=[0, 0], width=200, height=100):
            dpg.add_button(label="Request Auto Startup", tag=self._auto_startup_shutdown_tag, callback=self._run_auto_startup_shutdown)
            dpg.add_button(label="Emergency Stop", callback=self._emergency_stop)


        with dpg.window(label="Manual Startup/Shutdown", tag="manual_startup_shutdown", pos=[0, 100], width=400, height=200):
            dpg.add_button(label="Request Safe Power Up", tag=self._safe_power_up_down_tag, callback=self._run_safe_power_up_down)
            self._clear_faults = Button("Clear Faults")
            self._enable_actuators = CheckBox("Enable Actuators", False)
            dpg.add_button(label="Servo Robot", tag="servo_robot", callback=self._start_servo_unservo)
            with dpg.group(horizontal=True):
                dpg.add_text("Requested Master Gain: ")
                self._requested_master_gain = FloatSlider("Requested Master Gain", use_name_as_label=False, width=150)
            with dpg.group(horizontal=True):
                dpg.add_text("Control State: ")
                dpg.add_combo(items=[DO_NOTHING, USER_CONTROL], tag="control_state", width=100, default_value=DO_NOTHING)

    def _emergency_stop(self):
        self._unservo_quickly = True
        self._enable_actuators.set(False)
        self._requested_master_gain.set(0.0)
        self._request_auto_shutdown = True

    def _initialize_state_buttons(self):
        with dpg.window(label="Robot State", tag="robot_state", pos=[0, 300], width=400, height=250):
            with dpg.group(horizontal=True):
                dpg.add_text("Time")
                self._time = ValueDisplay("Time", initial_value=0.0)
            with dpg.group(horizontal=True):
                dpg.add_text("Low Level Master Gain")
                self._current_ll_master_gain = ValueDisplay("Current Master Gain", initial_value=0.0)
            self._is_faulted = CheckBox("Robot Faulted", False)
            self._are_actuators_enabled = CheckBox("Actuators Enabled", False)
            self._is_servoed = CheckBox("Robot Servoed", False)
            with dpg.group(horizontal=True):
                self._is_servoing = CheckBox("Servoing Robot", False)
                self._is_unservoing = CheckBox("Unservoing Robot", False)
            with dpg.group(horizontal=True):
                self._safe_power_up_complete = CheckBox("Safe Power Up Complete", False)
                self._safe_power_down_complete = CheckBox("Safe Power Down Complete", False)
            with dpg.group(horizontal=True):
                self._auto_startup_complete = CheckBox("Auto Startup Complete", False)
                self._auto_shutdown_complete = CheckBox("Auto Shut Down Complete", True)

    def _initialize_pose_buttons(self):
        self._wave_pose_state = 0
        self._pose_running = False
        self._pose_start_time = time.time()
        self._pose_duration = 3.0
        self._pose_joints = []
        joint_names = self._arm_control.joint_names
        self._initial_positions = [self._arm_control.get_desired_joint_position_by_name(name) for name in joint_names]
        self._final_positions = [self._arm_control.get_desired_joint_position_by_name(name) for name in joint_names]
        self._curr_positions = [self._arm_control.get_desired_joint_position_by_name(name) for name in joint_names]
        self._curr_pose = HOME_POSE

        with dpg.window(label="Poses", pos=[200, 0], width=200, height=100):
            with dpg.group(horizontal=True):
                dpg.add_text("Pose: ")
                dpg.add_combo(items=poses, tag="poses", width=100, default_value=HOME_POSE)

            self._run_pose = CheckBox("Run Pose", False)

    def _run_poses(self):
        if self._run_pose.value:
            if not self._pose_running:
                self._curr_pose = dpg.get_value("poses")
                self._pose_start_time = time.time()
                self._pose_running = True
                if self._curr_pose == HOME_POSE:
                    self._pose_duration = 5.0
                    self._pose_joints = home_pose_joints
                    self._final_positions = home_pose_values
                elif self._curr_pose == ARMS_UP_POSE:
                    self._pose_duration = 3.0
                    self._pose_joints = arms_up_pose_joints
                    self._final_positions = arms_up_pose_values
                elif self._curr_pose == WAVE_POSE:
                    self._pose_joints = wave_pose_joints
                    self._final_positions = wave_pose[self._wave_pose_state]
                elif self._curr_pose == DOUBLE_WAVE_POSE:
                    self._pose_joints = double_wave_pose_joints
                    self._final_positions = double_wave_pose[self._wave_pose_state]
                self._initial_positions = [self._arm_control.get_desired_joint_position_by_name(name) for name in self._pose_joints]
                print("Starting move to home pose")
            else:
                elapsed_time = time.time() - self._pose_start_time
                if elapsed_time < self._pose_duration:
                    update_pose_command(elapsed_time, self._pose_duration, self._initial_positions, self._final_positions, self._curr_positions)
                    for i in range(len(self._final_positions)):
                        self._arm_control.set_desired_joint_position_by_name(self._pose_joints[i], self._curr_positions[i])
                else:
                    for i in range(len(self._final_positions)):
                        self._arm_control.set_desired_joint_position_by_name(self._pose_joints[i], self._final_positions[i])

                    if (self._curr_pose == WAVE_POSE or self._curr_pose == DOUBLE_WAVE_POSE) and self._wave_pose_state < 3:
                        self._pose_duration = 1.0
                        self._wave_pose_state += 1
                        print("Switching to state " + str(self._wave_pose_state))
                        self._initial_positions[:] = self._final_positions[:]
                        if self._curr_pose == WAVE_POSE:
                            self._final_positions = wave_pose[self._wave_pose_state]
                        else:
                            self._final_positions = double_wave_pose[self._wave_pose_state]
                        print(self._initial_positions)
                        print(self._final_positions)
                        self._pose_start_time = time.time()
                    else:
                        self._run_pose.set(False)
                        self._pose_running = False
                        print(self._run_pose.value)
                        self._wave_pose_state = 0

                    print("Completed move to home pose")
                self._arm_control.send_desireds()


    def update_gui(self):
        self._update_auto_startup_shutdown()
        self._update_safe_power_up_down()
        if not self._high_level_servo_complete:
            self._run_servo_unservo()
        self._run_poses()
        dpg.render_dearpygui_frame()


    def _reset_sliders(self):
        self._arm_control.reset_sliders()
        self._arm_control.send_desireds()

    def run_gui(self, lock: Union[Lock, None] = None, shared_data: Union[Dict[str, Any], None] = None):
        while dpg.is_dearpygui_running():
            prev_loop_start_time = self._loop_start_time
            self._loop_start_time = time.perf_counter_ns()
            self._avg_loop_time += (self._loop_start_time - prev_loop_start_time) *1.0e-9
            self._loop_num += 1
            self.update_gui()
            if lock is not None and shared_data is not None:
                with lock:
                    self._read_state(shared_data["alex_state"])
                    self._write_command(shared_data["alex_command"])
                    self._hand_control.update_hands(shared_data)


            elapsed_time = (time.perf_counter_ns() - self._loop_start_time) * 1.0e-9
            # if self._loop_num >= self._loops_to_avg:
            #     avg_loop_time = self._avg_loop_time / self._loop_num
            #     print("Avg loop time: ", avg_loop_time)
            #     print("Avg freq: ", 1.0/ avg_loop_time)
            #     print("Percent missed loops: ", 100 * self._missed_loops / self._loop_num)
            #     self._avg_loop_time = 0.0
            #     self._loop_num = 0
            #     self._missed_loops = 0
            #     print(elapsed_time)
            if elapsed_time < self.dt:
                time.sleep(self.dt - elapsed_time)
            else:
                self._missed_loops += 1
                # print("Missed control loop")

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

    def _write_command(self, alex_command: AlexCommand):
        alex_command.request_auto_startup = self._request_auto_startup
        alex_command.request_auto_shutdown = self._request_auto_shutdown
        alex_command.clear_faults = self._clear_faults.value
        self._clear_faults.set(False)
        alex_command.request_enable_actuators = self._enable_actuators.value
        alex_command.request_disable_actuators = not self._enable_actuators.value
        alex_command.requested_master_gain = self._requested_master_gain.value
        alex_command.use_requested_master_gain = True
        alex_command.robot_control_state = robot_control_state[self._robot_control_state]

        self._arm_control.update_desireds(alex_command.joint_commands)

    def _start_servo_unservo(self):
        self._servo_robot = not self._servo_robot
        if not self._servo_robot:
            dpg.configure_item("servo_robot", label="Unservoing Robot", enabled=False)
            self._high_level_servo_complete = False
            self._servo_timer = time.time()
            self._servo_initial_value = self._current_ll_master_gain.value
        else:
            dpg.configure_item("servo_robot", label="Servoing Robot, press to cancel", enabled=True)
            self._servo_initial_value = 0.0
            self._servo_timer = time.time()
            self._high_level_servo_complete = False


    def _run_servo_unservo(self):
        servo_ratio = (time.time() - self._servo_timer) / 2.0

        # print(servo_ratio)
        if servo_ratio < 1.0:
            servo_ratio = round(servo_ratio, 2)
            if self._servo_robot:
                # self._master_gain_scale.set(servo_ratio)
                self._requested_master_gain.set(servo_ratio)
            else:
                # self._master_gain_scale.set(self._servo_initial_value* (1.0 - servo_ratio))
                self._requested_master_gain.set(self._servo_initial_value * (1.0 - servo_ratio))
            # print(self._requested_master_gain.get())
        else:
            self._high_level_servo_complete = True
            if self._servo_robot:
                dpg.configure_item("servo_robot", label="Unservo Robot", enabled=True)
                self._requested_master_gain.set(1.0)
                print("Robot is servoed")
            else:
                dpg.configure_item("servo_robot", label="Servo Robot", enabled=True)
                self._requested_master_gain.set(0.0)
                print("Robot is unservoed")



    def _run_auto_startup_shutdown(self):
        if not self._request_auto_startup and not self._request_auto_shutdown:
            if not self._auto_shutdown_complete and not self._auto_startup_complete:
                dpg.configure_item(self._auto_startup_shutdown_tag, label="Starting up, press to stop", enabled=True)
                self._request_auto_startup = True
                self._request_auto_shutdown = False
                print("Requesting Auto Startup")
            elif self._auto_startup_complete.value:
                dpg.configure_item(self._auto_startup_shutdown_tag, label="Shutting Down", enabled=False)
                self._request_auto_shutdown = True
                self._auto_startup_complete.set(False)
                print("Shutting down")
            elif self._auto_shutdown_complete.value:
                dpg.configure_item(self._auto_startup_shutdown_tag, label="Starting up, press to stop", enabled=True)
                self._request_auto_startup = True
                self._auto_shutdown_complete.set(False)
                print("Starting up")
        elif self._request_auto_startup:
            dpg.configure_item(self._auto_startup_shutdown_tag, label="Shutting Down", enabled=False)
            self._request_auto_shutdown = True
            self._request_auto_startup = False
            print("Shutting down midway through startup")
        if self._request_auto_startup:
            self._reset_sliders()
        self._begin_time = time.perf_counter_ns()
        print(self._request_auto_startup)

    def _update_auto_startup_shutdown(self):
        if self._request_auto_startup and self._auto_startup_complete.value:
            dpg.configure_item(self._auto_startup_shutdown_tag, label="Request Auto Shutdown", enabled=True)
            self._request_auto_startup = False
            self._enable_actuators.set(True)
            self._start_servo_unservo()
            self._robot_control_state = USER_CONTROL
            print("auto startup complete")
        elif self._request_auto_shutdown and self._auto_shutdown_complete.value:
            dpg.configure_item(self._auto_startup_shutdown_tag, label="Request Auto Startup", enabled=True)
            self._request_auto_shutdown = False
            self._enable_actuators.set(False)
            self._robot_control_state = DO_NOTHING
            print("auto shutdown complete")

    def _run_safe_power_up_down(self):
        if not self._request_safe_startup and not self._request_safe_shutdown:
            if not self._safe_power_down_complete.value and not self._safe_power_up_complete.value:
                dpg.configure_item(self._safe_power_up_down_tag, label="Powering up, press to stop", enabled=True)
                self._request_safe_startup = True
                self._request_safe_shutdown = False
            elif self._safe_power_up_complete.value:
                dpg.configure_item(self._safe_power_up_down_tag, label="Powering Down", enabled=False)
                self._request_safe_shutdown = True
                self._safe_power_up_complete = False
                print("Shutting down")
            elif self._safe_power_down_complete.value:
                dpg.configure_item(self._safe_power_up_down_tag, label="Powering up, press to stop", enabled=True)
                self._request_safe_startup = True
                self._safe_power_down_complete.set(False)
                print("Starting up")
            elif self._request_safe_startup:
                dpg.configure_item(self._safe_power_up_down_tag, label="Powering Down", enabled=False)
                self._request_safe_shutdown = True
                self._request_safe_startup = False
                print("Shutting down midway through startup")
        self._begin_time = time.perf_counter_ns()

    def _update_safe_power_up_down(self):
        if self._request_safe_startup and self._safe_power_up_complete.value:
            dpg.configure_item(self._safe_power_up_down_tag, label="Request Safe Power Down", enabled=True)
            self._request_safe_startup = False
            print("safe startup complete")
        elif self._request_safe_shutdown and self._safe_power_down_complete.value:
            dpg.configure_item(self._safe_power_up_down_tag, label="Request Safe Power Up", enabled=True)
            self._request_safe_shutdown = False
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
    dpg.create_context()
    dpg.configure_app(docking=True, docking_space=True)
    path = "../ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/hehe.urdf"
    robot = RobotModel.from_urdf(path)
    print('a')
    gui = AlexControlGUI(robot)
    print('b')
    dpg.create_viewport(title='Control GUI', width=900, height=1000, vsync=False)
    dpg.setup_dearpygui()
    print('c')
    dpg.show_viewport()

    gui.run_gui()
    dpg.destroy_context()




