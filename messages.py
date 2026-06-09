from calendar import day_abbr
from dataclasses import dataclass
from typing import Dict

from cyclonedds.core import Listener
from cyclonedds.idl.types import bounded_str, float64, int32, uint32, byte, array, sequence
from cyclonedds.idl import IdlStruct
from cyclonedds.sub import DataReader

@dataclass
class IMUState(IdlStruct):
    sensor_name: bounded_str[32]
    quaternion: array[float64, 4]
    gyroscope: array[float64, 3]
    accelerometer: array[float64, 3]
    temperature: int32
    is_operational: bool

@dataclass
class OneDOFJointCommand(IdlStruct):
    joint_name: bounded_str[35]
    q_des: float64 = 0.0
    qd_des: float64 = 0.0
    tau_des: float64 = 0.0
    stiffness: float64 = 0.0
    damping: float64 = 0.0
    max_position_error: float64 = 0.0
    max_velocity_error: float64 = 0.0
    max_torque: float64 = 0.0
    enable: bool = False
    joint_control_type: byte = 0

@dataclass
class OneDOFJointState(IdlStruct):
    joint_name: bounded_str[35]
    q: float64 = 0.0
    qd: float64 = 0.0
    tau: float64 = 0.0
    act_temp: float64 = 0.0
    is_operational: bool = 0

@dataclass
class ForceTorqueState(IdlStruct):
    sensor_name: bounded_str[32]
    force: array[float64, 3]
    torque: array[float64, 3]
    is_operational: bool

@dataclass
class AlexState(IdlStruct):
    time: float64
    is_faulted: bool
    is_calibrated: bool
    is_servoing: bool
    is_unservoing: bool
    is_servoed: bool
    are_actuators_enabled: bool
    safe_power_up_complete: bool
    safe_power_down_complete: bool
    auto_startup_complete: bool
    auto_shutdown_complete: bool
    current_low_level_master_gain: float64
    joint_states: sequence[OneDOFJointState, 50]
    number_of_joints: uint32
    imu_states: sequence[IMUState, 50]
    number_of_imus: uint32
    ft_states: sequence[ForceTorqueState, 50]
    number_of_fts: uint32

@dataclass
class AlexCommand(IdlStruct):
    request_auto_startup: bool
    request_auto_shutdown: bool
    request_safe_power_up: bool
    request_safe_power_down: bool
    request_enable_actuators: bool
    request_disable_actuators: bool
    clear_faults: bool
    calibrate: bool
    servo_actuators: bool
    unservo_quickly: bool
    use_requested_master_gain: bool
    requested_master_gain: float64
    disable_noncritical_faults: bool
    robot_control_state: byte
    joint_commands: sequence[OneDOFJointCommand, 50]
    number_of_joints: uint32

@dataclass
class FortRoboticsRCHandheldState(IdlStruct):
    # Joystick states
    left_joystick_x_normalized: float64
    left_joystick_y_normalized: float64
    right_joystick_x_normalized: float64
    right_joystick_y_normalized: float64
    # Trigger states
    left_trigger_normalized: float64
    right_trigger_normalized: float64
    # button 1-4
    button1_pressed: bool
    button2_pressed: bool
    button3_pressed: bool
    button4_pressed: bool
    # D-Pad
    button_up_pressed: bool
    button_down_pressed: bool
    button_left_pressed: bool
    button_right_pressed: bool
    # E-Stop
    e_stop_pressed: bool
    # Battery
    battery_level: float64

@dataclass
class HardwareResources(IdlStruct):
    num_xml_resources: uint32
    num_urdf_resources: uint32
    xml_resources: sequence[bounded_str[32], 11]
    urdf_resources: sequence[bounded_str[32], 10]
    directory: bounded_str[32]

@dataclass
class ROSDeviceStatusProvider(IdlStruct):
    name: bounded_str[70]
    is_responding: bool
    is_faulted: bool
    ethercat_state: byte

    under_voltage: bool
    over_voltage: bool
    sto_disabled: bool
    current_short: bool
    over_temp: bool

@dataclass
class HardwareStatus(IdlStruct):
    robot_fault: bool
    motor_fault: bool
    missed_deadline_fault: bool
    working_counter_fault: bool
    bus_over_voltage_fault: bool
    bus_over_current_fault: bool

    working_counter_mismatch_count: uint32
    missed_deadlines: uint32

    battery_charge_percetage: float64
    estimated_runtime_minutes: uint32
    bus_over_voltage_warning: bool
    bus_over_current_warning: bool
    battery_voltage_volts: float64
    battery_current_amps: float64
    battery_power_watts: float64
    power_supply_voltage_volts: float64
    power_supply_current_amps: float64
    power_supply_power_watts: float64
    motor_bus_voltage_volts: float64
    motor_bus_current_amps: float64
    motor_bus_power_watts: float64

    device_status_providers: sequence[ROSDeviceStatusProvider, 75]




