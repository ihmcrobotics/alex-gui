# from alex_communication2 import AlexCommunication
from alex_visualizer import AlexVisualizer
from alex_control_gui import AlexControlGUI
import threading
import time
import numpy as np

from messages import OneDOFJointCommand, OneDOFJointState

thread_lock = threading.Lock()

path = "../ihmc-alex-sdk/alex-models/alex_purdue_description/urdf/hehe.urdf"

visualizer = AlexVisualizer(path)
print('a')
robot = visualizer.get_robot_model()
print('b')
shared_data = {"joint_desired_positions": np.zeros(len(robot.joint_names)),
               "joint_commands": {name: OneDOFJointCommand(joint_name=name) for name in robot.joint_names},
               "joint_states": {name: OneDOFJointState(joint_name=name) for name in robot.joint_names}}
gui = AlexControlGUI(robot.joint_names, robot.joint_min_angles, robot.joint_max_angles)
# communication = AlexCommunication()
print('c')
vis_thread = threading.Thread(target=visualizer.run_visualizer, args=(thread_lock, shared_data), daemon=True)
# comm_thread = threading.Thread(target=communication.run_communication, args=(thread_lock, shared_data), daemon=True)
# gui_thread = threading.Thread(target=gui.run_gui, args=(thread_lock, shared_data), daemon=True)
print('e')
vis_thread.start()
print('f')
# comm_thread.start()
# vis_thread.join()
# gui_thread.start()
print('g')
gui.run_gui(thread_lock, shared_data)
print('h')
# vis_thread.join()