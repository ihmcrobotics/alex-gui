import threading
from threading import Lock

from alex_communication import AlexCommunication
from alex_control_gui import AlexControlGUI
from alex_visualizer import AlexVisualizer
from messages import OneDOFJointCommand, OneDOFJointState, AlexState, AlexCommand
import logging

def main(urdf_path: str):
    thread_lock = Lock()

    robot_visualizer = AlexVisualizer(urdf_path)
    robot = robot_visualizer.get_robot_model()
    control_gui = AlexControlGUI(robot)
    communication = AlexCommunication()
    joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_names]
    alex_command = AlexCommand(joint_commands=joint_commands)

    shared_data = {"joint_commands": {name: OneDOFJointCommand(joint_name=name) for name in robot.joint_names},
                   "joint_states": {},
                   "hardware_status": None,
                   "alex_state": None,
                   "alex_command": alex_command}

    visual_thread = threading.Thread(target=robot_visualizer.run_visualizer, args=(thread_lock, shared_data), daemon=True)
    print('Starting visualizer...')
    visual_thread.start()

    comms_thread = threading.Thread(target=communication.run_communication, args=(thread_lock, shared_data), daemon=True)
    print('Starting communication...')
    comms_thread.start()

    print('Starting GUI...')
    control_gui.run_gui(thread_lock, shared_data)

if __name__ == '__main__':
    logging.getLogger("skrobot").setLevel(logging.ERROR)
    path = "../ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/hehe.urdf"
    main(path)