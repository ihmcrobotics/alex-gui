"""
This file is the main file of alex-gui, used to run the example communication, visualization, and control of Alex at the
high level.
"""

import logging
import threading
from threading import Lock

import dearpygui.dearpygui as dpg
from screeninfo import get_monitors

from communication import AlexCommunication
from communication.messages import *
from gui import AlexControlGUI
from visualizer import AlexVisualizer
from visualizer.urdf_helper import merge_urdfs, PURDUE_FULL_PARTS


def main(urdf_path: str):
    # Check the monitor settings and alter the scale depending
    monitor = get_monitors()[0]
    if monitor.width < 2000:
        monitor_scale = 1.0
    else: 
        monitor_scale = 2.0
    height, width = monitor.height, monitor.width

    # Set the update frequencies for each part
    control_frequency = 100.0
    comm_frequency = 100.0
    vis_frequency = 50.0

    # Initialize the visualizer
    robot_visualizer = AlexVisualizer(urdf_path, frequency = vis_frequency)

    # Initialize the control GUI
    robot = robot_visualizer.get_robot_model()
    control_gui = AlexControlGUI(robot, frequency=control_frequency, monitor_scale=monitor_scale, height=int(0.9*height), width=int(3*width/4)) #, width=int(width/2), height=height)

    # Initialize the communication. If just wanting to communicate with the robot, make sure the IP address is set to
    # the address that connects to the robot. Otherwise, 127.0.0.1 will allow the GUI and visualizer to still run
    communication = AlexCommunication("10.43.3.3", frequency=comm_frequency) #"10.43.3.3"
    joint_commands = [OneDOFJointCommand(joint_name=name) for name in robot.joint_names]
    alex_command = AlexCommand(joint_commands=joint_commands, number_of_joints=len(joint_commands))

    # Define the dictionary of shared date
    shared_data = {"hardware_status": HardwareStatus(),
                   "alex_state": AlexState(),
                   "alex_command": alex_command,
                   "left_hand_state": EZGripperState(),
                   "right_hand_state": EZGripperState(),
                   "left_hand_command": EZGripperCommand(),
                   "right_hand_command": EZGripperCommand()}

    # Set up the threading to run the visual and communication thread in parallel with the control GUI
    thread_lock = Lock()
    visual_thread = threading.Thread(target=robot_visualizer.run_visualizer, args=(thread_lock, shared_data), daemon=True)
    comms_thread = threading.Thread(target=communication.run_communication, args=(thread_lock, shared_data), daemon=True)
    print('Starting visualizer...')
    visual_thread.start()
    print('Starting communication...')
    comms_thread.start()

    # Run the control GUI
    print('Starting GUI...')
    dpg.show_viewport()
    control_gui.run_gui(thread_lock, shared_data)
    dpg.destroy_context()

if __name__ == '__main__':
    logging.getLogger("skrobot").setLevel(logging.ERROR)
    # Create the full urdf for visualization and control
    path = merge_urdfs("purdue", PURDUE_FULL_PARTS, fixed_joints=["PEDESTAL_F"], save_name="purdue_full")
    main(path)