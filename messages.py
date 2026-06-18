import math
from dataclasses import dataclass, field
from cyclonedds.idl.types import bounded_str, float64, int32, uint32, byte, array, sequence, uint8, uint64, float32
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

# @dataclass
# class HardwareResources(IdlStruct):
#     num_xml_resources: uint32 = 0
#     num_urdf_resources: uint32 = 0
#     xml_resources: sequence[str, 11] = field(default_factory=list)
#     urdf_resources: sequence[str, 10] = field(default_factory=list)
#     directory: str = ""

@dataclass
class ROSDeviceStatusProvider(IdlStruct, typename="alex_msgs::msg::dds_::ROSDeviceStatusProvider_"):
    # The device name
    name: bounded_str[70] = ""
    # Primary device status info
    is_responding: bool = False
    is_faulted: bool = False
    ethercat_state: uint8 = 0

    # More specific device status info related to isFaulted (elmo twitter fault)
    under_voltage: bool = False
    over_voltage: bool = False
    sto_disabled: bool = False
    current_short: bool = False
    over_temp: bool = False

@dataclass
class HardwareStatus(IdlStruct, typename="alex_msgs::msg::dds_::HardwareStatus_"):
    # Various fault info
    robot_fault: bool = False
    motor_fault: bool = False
    missed_deadline_fault: bool = False
    working_counter_fault: bool = False
    bus_over_voltage_fault: bool = False
    bus_over_current_fault: bool = False

    # Important EtherCAT Stuff
    working_counter_mismatch_count: uint32 = 0
    missed_deadlines: uint32 = 0

    # Power Management Board Information
    battery_charge_percetage: float64 = 0.0
    estimated_runtime_minutes: float64 = 0.0
    bus_over_voltage_warning: bool = False
    bus_over_current_warning: bool = False
    battery_voltage_volts: float64 = 0.0
    battery_current_amps: float64 = 0.0
    battery_power_watts: float64 = 0.0
    power_supply_voltage_volts: float64 = 0.0
    power_supply_current_amps: float64 = 0.0
    power_supply_power_watts: float64 = 0.0
    motor_bus_voltage_volts: float64 = 0.0
    motor_bus_current_amps: float64 = 0.0
    motor_bus_power_watts: float64 = 0.0

    # Joint States
    device_status_providers: sequence[ROSDeviceStatusProvider, 75] = field(default_factory=list)

@dataclass
class EZGripperState(IdlStruct, typename="ihmc_hands_ros2::msg::dds_::EZGripperState_"):
    # Specifies the current operation mode.
    operation_mode: uint8 = 255
    # Temperature of the Dynamixel in Celsius
    temperature: uint8 = 0
    # The current Dynamixel position
    # 0.0 = fully closed, 1.0 = fully open
    current_position: float32 = 0.0
    # The current amount of effort being used
    # 0.0 = no effort, 1.0 = maximum effort
    current_effort: float32 = 0.0
    # Dynamixel's error code
    # See: https://emanual.robotis.com/docs/en/dxl/protocol1/#error
    error_code: uint8 = 0
    # Realtime tick of the Dynamixel
    # If this value isn't changing, communication with the gripper is broken
    realtime_tick: int32 = 0
    # Whether the hand has been calibrated
    is_calibrated: bool = False

@dataclass
class EZGripperCommand(IdlStruct, typename="ihmc_hands_ros2::msg::dds_::EZGripperCommand_"):
    operation_mode: uint8 = 0
    temperature_limit: uint8 = 75
    goal_position: float32 = 0.0
    max_effort: float32 = 0.3
    torque_on: bool = False

@dataclass
class HandJointAnglePacket(IdlStruct, typename="controller_msgs::msg::dds_::HandJointAnglePacket_"):
    sequence_id: uint64 = 0
    robot_side: uint8 = 0
    joint_angles: sequence[float64] = field(default_factory=list)
    connected: bool = False
    calibrated: bool = False


