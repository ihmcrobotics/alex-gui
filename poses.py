from typing import List

home_pose_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
home_pose_joints = [
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
    "LEFT_WRIST_X",
    "LEFT_WRIST_Z",
    "LEFT_GRIPPER_Y",
    "RIGHT_SHOULDER_X",
    "RIGHT_SHOULDER_Y",
    "RIGHT_SHOULDER_Z",
    "RIGHT_ELBOW_Y",
    "RIGHT_WRIST_X",
    "RIGHT_WRIST_Z",
    "RIGHT_GRIPPER_Y",
    "NECK_Z",
    "NECK_Y"
]

arms_up_pose_values = [0.4, -0.8, 0.4, -1.9, 0.0, 0.0, 0.0, -0.4, -0.8, -0.4, -1.9, 0.0, 0.0, 0.0, 0.0, 0.0]
arms_up_pose_joints = [
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
    "LEFT_WRIST_X",
    "LEFT_WRIST_Z",
    "LEFT_GRIPPER_Y",
    "RIGHT_SHOULDER_X",
    "RIGHT_SHOULDER_Y",
    "RIGHT_SHOULDER_Z",
    "RIGHT_ELBOW_Y",
    "RIGHT_WRIST_X",
    "RIGHT_WRIST_Z",
    "RIGHT_GRIPPER_Y",
    "NECK_Z",
    "NECK_Y"
]

wave_pose = [[1.5, -0.6, 1.0, -1.5],
             [1.5, -0.6, 1.0, -1.0],
             [1.5, -0.6, 1.0, -2.0],
             [1.5, -0.6, 1.0, -1.5]]
wave_pose_joints = [
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y"
]

double_wave_pose = [[1.5, -0.6, 1.0, -1.5],
             [1.5, -0.6, 1.0, -1.0],
             [1.5, -0.6, 1.0, -2.0],
             [1.5, -0.6, 1.0, -1.5]]
double_wave_pose_joints = [
    "LEFT_SHOULDER_X",
    "LEFT_SHOULDER_Y",
    "LEFT_SHOULDER_Z",
    "LEFT_ELBOW_Y",
]

def update_pose_command(run_time: float, duration: float, initial_positions: List[float], final_positions: List[float], desired_positions: List[float]):
    for joint in range(len(final_positions)):
        desired_positions[joint] = initial_positions[joint] + run_time / duration * (final_positions[joint] - initial_positions[joint])