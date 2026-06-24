import math
import time

from skrobot.model import RobotModel

from communication.messages import OneDOFJointState, OneDOFJointCommand
from .gui_helpers import *


def initialize_joint_position_sliders(joint_names: List[str],
                                      lower_limits: List[float] | None = None,
                                      upper_limits: List[float] | None = None) -> Dict[str, FloatSlider]:
    """
    Initialize the desired joint position sliders.
    :param joint_names: List of joint names for each slider
    :param lower_limits: List of lower limits for each slider
    :param upper_limits: List of upper limits for each slider
    :return: Dictionary of sliders tied to each joint name
    """
    joint_sliders = {}
    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        joint_sliders[joint_name] = FloatSlider(joint_name, min_value=lower_limits[i], max_value=upper_limits[i])
    return joint_sliders


def initialize_joint_settings_tabs(joint_settings: Dict[str, JointSettings], monitor_scale: float = 1.0) -> None:
    """
    Initialize the joint settings tabs

    :param joint_settings: Dictionary of joint settings
    :param monitor_scale: Scaling for dearpygui items based on the monitor being used
    """
    with dpg.tab_bar():
        with dpg.tab(label="Impedance"):
            with dpg.table(header_row=True):
                dpg.add_table_column(label="Joint")
                dpg.add_table_column(label="Stiffness")
                dpg.add_table_column(label="Damping")

                for name, settings in joint_settings.items():
                    with dpg.table_row():
                        dpg.add_text(name)
                        dpg.add_input_float(tag=name + "_stiffness",
                                            default_value=settings.stiffness,
                                            width=int(100 * monitor_scale))
                        dpg.add_input_float(tag=name + "_damping",
                                            default_value=settings.damping,
                                            width=int(100 * monitor_scale))

        with dpg.tab(label="Limits"):
            with dpg.table(header_row=True):
                dpg.add_table_column(label="Joint")
                dpg.add_table_column(label="Max Torque")
                dpg.add_table_column(label="Max Pos Error")
                dpg.add_table_column(label="Max Vel Error")
                for name, settings in joint_settings.items():
                    with dpg.table_row():
                        dpg.add_text(name)
                        dpg.add_input_float(tag=name + "_max_torque",
                                            default_value=settings.max_torque,
                                            width=int(100 * monitor_scale))
                        dpg.add_input_float(tag=name + "_max_position_error",
                                            default_value=settings.max_position_error,
                                            width=int(100 * monitor_scale))
                        dpg.add_input_float(tag=name + "_max_velocity_error",
                                            default_value=settings.max_velocity_error,
                                            width=int(100 * monitor_scale))


class JointControlGUI:
    """
    This class holds all the windows for controlling all the robot joints
    """

    def __init__(self, robot: RobotModel, monitor_scale: float = 1.0) -> None:
        """
        Initialize the GUI
        :param robot: Robot model to take initial joint data from
        :param monitor_scale: Scaling for dearpygui items based on the monitor being used
        """
        self.robot = robot

        self.settings_dimensions = [0, 0]
        self.control_dimensions = [0, 0]

        self.joint_names = []
        self.joint_list = []
        self.joint_lower_limits = []
        self.joint_upper_limits = []
        self.joint_max_torques = []
        self.joint_settings = {}

        # Exclude hand joints as they are controlled separately
        for joint in robot.joint_list:
            if "ezgripper" not in joint.name:
                self.joint_names.append(joint.name)
                self.joint_list.append(joint)
                self.joint_lower_limits.append(joint.min_joint_angle)
                self.joint_upper_limits.append(joint.max_joint_angle)
                self.joint_max_torques.append(joint.max_joint_torque)
                self.joint_settings[joint.name] = JointSettings(max_torque=joint.max_joint_torque)

        self.measured_joint_positions = {joint: 0.0 for joint in self.joint_names}

        # Create and populate the joint control window
        with dpg.window(label="Joint Control", tag="joint_control"):
            with dpg.group(horizontal=True):
                self._send_desireds = Button("Send Desireds")
                self._send_desireds_continuously = CheckBox("Send Desireds Continuously", initial_value=True)
            dpg.add_button(label="Reset sliders", callback=self.reset_sliders)
            self._desired_joint_positions = initialize_joint_position_sliders(self.joint_names,
                                                                              lower_limits=self.joint_lower_limits,
                                                                              upper_limits=self.joint_upper_limits)

        # Create and populate the joint settings window
        with dpg.window(label="Joint Settings", tag="joint_settings"):
            with dpg.group(horizontal=True):
                self._update_settings_button = Button("Update Settings")
                self._update_settings_continuously = CheckBox("Update Settings Continuously")
            self._use_custom_impedance = CheckBox("Use Custom Impedance", initial_value=True)
            initialize_joint_settings_tabs(self.joint_settings, monitor_scale=monitor_scale)

    def reset_sliders(self):
        """
        Reset all sliders to the measured joint positions of the robot
        """
        for name in self.joint_names:
            self._desired_joint_positions[name].set(self.measured_joint_positions[name])
        self._send_desireds.set(True)

    def updated_measured(self, joint_states: List[OneDOFJointState]):
        """
        Updated the measured joint positions
        :param joint_states: List of joint states
        """
        for joint_state in joint_states:
            name = joint_state.joint_name
            if name in self.joint_names:
                self.measured_joint_positions[name] = joint_state.q

    def _update_settings(self):
        """
        Update the joint settings from the joint settings tab
        """
        for name in self.joint_names:
            # Grab the new desired joint settings
            joint_setting = self.joint_settings[name]
            desired_stiffness = dpg.get_value(name + "_stiffness")
            desired_damping = dpg.get_value(name + "_damping")
            desired_max_torque = dpg.get_value(name + "_max_torque")
            desired_max_position_error = dpg.get_value(name + "_max_position_error")
            desired_max_velocity_error = dpg.get_value(name + "_max_velocity_error")

            # Update the joint settings
            joint_setting.update_all_settings(desired_stiffness, desired_damping, desired_max_torque,
                                              desired_max_position_error, desired_max_velocity_error)

            # Add them back after they've been limited
            dpg.set_value(name + "_stiffness", joint_setting.stiffness)
            dpg.set_value(name + "_damping", joint_setting.damping)
            dpg.set_value(name + "_max_torque", joint_setting.max_torque)
            dpg.set_value(name + "_max_position_error", joint_setting.max_position_error)
            dpg.set_value(name + "_max_velocity_error", joint_setting.max_velocity_error)

    def _update_internal(self):
        """
        Complete an internal update for the joints. At current, this only includes joint settings
        """
        if self._update_settings_button.value or self._update_settings_continuously.value:
            self._update_settings()
            self._update_settings_button.set(False)

    def set_spacing(self):
        """
        Set the spacing of the joint control and joint settings windows based on the size of the viewport
        """
        self.control_dimensions = dpg.get_item_rect_size("joint_control")
        self.settings_dimensions = dpg.get_item_rect_size("joint_settings")
        width = dpg.get_viewport_client_width()
        dpg.set_item_pos("joint_control", [width - self.control_dimensions[0], 0])
        dpg.set_item_pos("joint_settings", [width - self.settings_dimensions[0], self.control_dimensions[1]])

    def update_desireds(self, commands: List[OneDOFJointCommand]):
        """
        Update the desireds of the robot, including desired joint positions and joint settings
        :param commands: List of joint commands to place the new desireds into
        """
        self._update_internal()
        if self._send_desireds.value or self._send_desireds_continuously.value:
            for command in commands:
                name = command.joint_name
                if name in self.joint_names:
                    command.q_des = self._desired_joint_positions[name].value
                    command.qd_des = 0.0
                    command.taw_des = 0.0
                    #If using custom impedance, add the new settings, else, set all to NaN to tell low level to use default
                    if self._use_custom_impedance.value:
                        command.stiffness = self.joint_settings[name].stiffness
                        command.damping = self.joint_settings[name].damping
                        command.max_torque = self.joint_settings[name].max_torque
                        command.max_position_error = self.joint_settings[name].max_position_error
                        command.max_velocity_error = self.joint_settings[name].max_velocity_error
                    else:
                        command.stiffness = math.nan
                        command.damping = math.nan
                        command.max_torque = math.nan
                        command.max_position_error = math.nan
                        command.max_velocity_error = math.nan
            self._send_desireds.set(False)

    def send_desireds(self):
        """
        Tell the controller to send desired commands
        """
        self._send_desireds.set(True)

    def get_desired_joint_position_by_name(self, name: str):
        return self._desired_joint_positions[name].value

    def set_desired_joint_position_by_name(self, name: str, value: float):
        self._desired_joint_positions[name].set(value)


if __name__ == "__main__":
    dpg.create_context()
    dpg.configure_app(docking=True, docking_space=True)
    path = "../../../ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/hehe.urdf"
    this_robot = RobotModel.from_urdf(path)
    print('a')
    arm_control = JointControlGUI(this_robot)
    print('b')

    dpg.create_viewport(title='Custom Title', width=500, height=700)
    dpg.setup_dearpygui()
    print('c')
    dpg.show_viewport()
    # dpg.set_primary_window("joint_control", True)
    while dpg.is_dearpygui_running():
        dpg.render_dearpygui_frame()
        arm_control._update_internal()
        time.sleep(0.01)
    print('d')
    dpg.start_dearpygui()
    print('e')
    dpg.destroy_context()
    print('f')
