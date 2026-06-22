import threading
from threading import Lock
import dearpygui.dearpygui as dpg

from alex_communication import AlexCommunication
from alex_control_gui import AlexControlGUI
from alex_robot_visualizer import AlexVisualizer
from messages import *
import logging
from urdf_helper import merge_urdfs, PURDUE_FULL_PARTS, PURDUE_CYCLOIDFOREARMS_PARTS
from screeninfo import get_monitors

def main(urdf_path: str):
    monitor = get_monitors()[0]
    height, width = monitor.height, monitor.width
    thread_lock = Lock()
    control_frequency = 100.0
    comm_frequency = 100.0
    vis_frequency = 200.0
    robot_visualizer = AlexVisualizer(urdf_path, frequency = vis_frequency)
    robot = robot_visualizer.get_robot_model()
    control_gui = AlexControlGUI(robot, frequency=control_frequency, width=int(width/2), height=height)
    communication = AlexCommunication("127.0.0.1", frequency=comm_frequency) #"10.100.4.183")
    joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_names]
    alex_command = AlexCommand(joint_commands=joint_commands, number_of_joints=len(joint_commands))

    shared_data = {"hardware_status": HardwareStatus(),
                   "alex_state": AlexState(),
                   "alex_command": alex_command,
                   "left_hand_state": EZGripperState(),
                   "right_hand_state": EZGripperState(),
                   "left_hand_command": EZGripperCommand(),
                   "right_hand_command": EZGripperCommand(),
                   "hand_angles": HandJointAnglePacket()}

    visual_thread = threading.Thread(target=robot_visualizer.run_visualizer, args=(thread_lock, shared_data), daemon=True)
    print('Starting visualizer...')
    visual_thread.start()

    comms_thread = threading.Thread(target=communication.run_communication, args=(thread_lock, shared_data), daemon=True)
    print('Starting communication...')
    comms_thread.start()

    print('Starting GUI...')
    dpg.show_viewport()
    # dpg.maximize_viewport()
    control_gui.run_gui(thread_lock, shared_data)
    dpg.destroy_context()

if __name__ == '__main__':
    logging.getLogger("skrobot").setLevel(logging.ERROR)
    path = merge_urdfs("purdue", PURDUE_FULL_PARTS, fixed_joints=["PEDESTAL_F"], output_name="purdue_full")
    # path = "../ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/hehe_full.urdf"
    main(path)