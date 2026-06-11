import math
from dataclasses import dataclass, field
from cyclonedds.idl.types import bounded_str, float64, int32, uint32, byte, array, sequence, uint8
from cyclonedds.idl import IdlStruct

@dataclass
class IMUState(IdlStruct, typename="alex_msgs::msg::dds_::IMUState_"):
    sensor_name: bounded_str[32]
    quaternion: array[float64, 4]
    gyroscope: array[float64, 3]
    accelerometer: array[float64, 3]
    temperature: int32
    is_operational: bool

@dataclass
class OneDOFJointCommand(IdlStruct, typename="alex_msgs::msg::dds_::OneDOFJointCommand_"):
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
class OneDOFJointState(IdlStruct, typename="alex_msgs::msg::dds_::OneDOFJointState_"):
    joint_name: bounded_str[35]
    q: float64 = 0.0
    qd: float64 = 0.0
    tau: float64 = 0.0
    act_temp: float64 = 0.0
    is_operational: bool = False

@dataclass
class ForceTorqueState(IdlStruct, typename="alex_msgs::msg::dds_::ForceTorqueState_"):
    sensor_name: bounded_str[32]
    force: array[float64, 3]
    torque: array[float64, 3]
    is_operational: bool

@dataclass
class AlexState(IdlStruct, typename="alex_msgs::msg::dds_::AlexState_"):
    time: float64 = 0.0
    is_faulted: bool = False
    is_calibrated: bool = False
    is_servoing: bool = False
    is_unservoing: bool = False
    is_servoed: bool = False
    are_actuators_enabled: bool = False
    safe_power_up_complete: bool = False
    safe_power_down_complete: bool = False
    auto_startup_complete: bool = False
    auto_shutdown_complete: bool = False
    current_low_level_master_gain: float64 = 0.0
    joint_states: sequence[OneDOFJointState, 50] = field(default_factory=list)
    number_of_joints: uint32 = 0
    imu_states: sequence[IMUState, 50]= field(default_factory=list)
    number_of_imus: uint32 = 0
    ft_states: sequence[ForceTorqueState, 50]= field(default_factory=list)
    number_of_fts: uint32 = 0

@dataclass
class AlexCommand(IdlStruct, typename="alex_msgs::msg::dds_::AlexCommand_"):
    request_auto_startup: bool = False
    request_auto_shutdown: bool = False
    request_safe_power_up: bool = False
    request_safe_power_down: bool = False
    request_enable_actuators: bool = False
    request_disable_actuators: bool = False
    clear_faults: bool = False
    calibrate: bool = False
    servo_actuators: bool = False
    unservo_quickly: bool = False
    use_requested_master_gain: bool = False
    requested_master_gain: float64 = 0.0
    disable_noncritical_faults: bool = False
    robot_control_state: uint8 = 0
    joint_commands: sequence[OneDOFJointCommand, 50] = field(default_factory=list)
    number_of_joints: uint32 = 0.0
#
# @dataclass
# class FortRoboticsRCHandheldState(IdlStruct):
#     # Joystick states
#     left_joystick_x_normalized: float64
#     left_joystick_y_normalized: float64
#     right_joystick_x_normalized: float64
#     right_joystick_y_normalized: float64
#     # Trigger states
#     left_trigger_normalized: float64
#     right_trigger_normalized: float64
#     # button 1-4
#     button1_pressed: bool
#     button2_pressed: bool
#     button3_pressed: bool
#     button4_pressed: bool
#     # D-Pad
#     button_up_pressed: bool
#     button_down_pressed: bool
#     button_left_pressed: bool
#     button_right_pressed: bool
#     # E-Stop
#     e_stop_pressed: bool
#     # Battery
#     battery_level: float64
#
# @dataclass
# class HardwareResources(IdlStruct):
#     num_xml_resources: uint32
#     num_urdf_resources: uint32
#     xml_resources: sequence[bounded_str[32], 11]
#     urdf_resources: sequence[bounded_str[32], 10]
#     directory: bounded_str[32]
#
# @dataclass
# class ROSDeviceStatusProvider(IdlStruct):
#     name: bounded_str[70]
#     is_responding: bool
#     is_faulted: bool
#     ethercat_state: byte

# @dataclass
# class AlexCommand(IdlStruct):
#     request_auto_startup: bool = False
#     request_auto_shutdown: bool = False
#     request_safe_power_up: bool = False
#     request_safe_power_down: bool = False
#     request_enable_actuators: bool = False
#     request_disable_actuators: bool = False
#     clear_faults: bool = False
#     calibrate: bool = False
#     servo_actuators: bool = False
#     unservo_quickly: bool = False
#     use_requested_master_gain: bool = False
#     requested_master_gain: float= 0.0
#     disable_noncritical_faults: bool = False
#     robot_control_state: byte = field(default_factory=byte)
#     joint_commands: sequence[OneDOFJointCommand, 50] = field(default_factory=list)
#     number_of_joints: uint32 = 0

# @dataclass
# class FortRoboticsRCHandheldState(IdlStruct):
#     # Joystick states
#     left_joystick_x_normalized: float64
#     left_joystick_y_normalized: float64
#     right_joystick_x_normalized: float64
#     right_joystick_y_normalized: float64
#     # Trigger states
#     left_trigger_normalized: float64
#     right_trigger_normalized: float64
#     # button 1-4
#     button1_pressed: bool
#     button2_pressed: bool
#     button3_pressed: bool
#     button4_pressed: bool
#     # D-Pad
#     button_up_pressed: bool
#     button_down_pressed: bool
#     button_left_pressed: bool
#     button_right_pressed: bool
#     # E-Stop
#     e_stop_pressed: bool
#     # Battery
#     battery_level: float64

# @dataclass
# class HardwareResources(IdlStruct):
#     num_xml_resources: uint32 = 0
#     num_urdf_resources: uint32 = 0
#     xml_resources: sequence[str, 11] = field(default_factory=list)
#     urdf_resources: sequence[str, 10] = field(default_factory=list)
#     directory: str = ""

@dataclass
class ROSDeviceStatusProvider(IdlStruct, typename="alex_msgs::msg::dds_::ROSDeviceStatusProvider_"):
    name: bounded_str[70]
    is_responding: bool
    is_faulted: bool
    ethercat_state: uint8

    under_voltage: bool
    over_voltage: bool
    sto_disabled: bool
    current_short: bool
    over_temp: bool

@dataclass
class HardwareStatus(IdlStruct, typename="alex_msgs::msg::dds_::HardwareStatus_"):
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




