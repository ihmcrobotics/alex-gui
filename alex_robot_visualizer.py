from typing import Literal, List, Dict
from skrobot.model import RobotModel, Link, CascadedLink
from skrobot.models import PR2
from skrobot.viewers import TrimeshSceneViewer, PyrenderViewer, ViserViewer
import random
import time
from threading import Lock


class AlexVisualizer:
    def __init__(self, urdf_path: str, ghost_path: str | None = None, frequency: float = 100.0):
        self.robot = RobotModel.from_urdf(urdf_path)
        self.ghost_robot = RobotModel.from_urdf(ghost_path if ghost_path is not None else urdf_path)
        self.ghost_robot.name = "ghost"
        self.frequency = frequency
        self.dt = 1.0/self.frequency
        self._joints = {joint.name: joint for joint in self.robot.joint_list}
        self._viewer = None

        for link in self.ghost_robot.link_list:
            link.set_alpha(0.5)
        self.initialized = False

    def initialize(self) -> None:
        self._viewer = PyrenderViewer(update_interval=self.dt)

        self._viewer.add(self.robot)
        self._viewer.add(self.ghost_robot)

        self._viewer.show()
        self._viewer.redraw()
        self.initialized = True

    def update(self, lock: Lock, data: Dict) -> None:
        self.curr_time = time.perf_counter_ns()
        if not self.initialized:
            self.initialize()
        else:
            with lock:
                # print("reading joint positions")
                # joint_desireds = data["joint_command"]
                command = data["alex_command"]
                for i in range(len(self.ghost_robot.joint_list)):
                    desired = command.joint_commands[i]
                    self.ghost_robot.joint_list[i].joint_angle(desired.q_des)

                joint_states = data["alex_state"].joint_states

                if len(joint_states) > 0:
                    for joint_state in joint_states:
                        joint_name = joint_state.joint_name
                        self._joints[joint_name].joint_angle(joint_state.q)

                # new_positions = data["joint_desired_positions"] #random.uniform(self.ghost_robot.joint_min_angles, self.ghost_robot.joint_max_angles)
                # print(new_positions)
            # print(new_positions)
            # self.ghost_robot.angle_vector(new_positions)
            self._viewer.redraw()

    def run_visualizer(self, lock: Lock, data: Dict):
        if not self.initialized:
            self.initialize()

        while True:
            self.update(lock, data)
            elapsed_time = time.perf_counter_ns() - self.curr_time
            remaining_time = self.dt - elapsed_time * 1.0e-9
            # run_slider_panel()
            if (remaining_time > 0):
                time.sleep(remaining_time)
            else:
                print("missed visualizer loop ")

    def get_joint_names(self) -> List[str]:
        return self.robot.joint_names

    def get_robot_model(self) -> RobotModel:
        return self.robot




if __name__ == "__main__":
    ns_to_s = 1.0e-9
    freq = 200
    dt = 1.0 / freq
    path = "/ihmc-alex-sdk/alex-models/alex_description/urdf/002/hehehe.urdf"

    visualizer = AlexVisualizer(path)
    visualizer.run_visualizer()
# desired_robot = RobotModel.from_urdf("/home/rpeterson/repos/python-repo/ihmc-alex-sdk/alex-models/alex_description/urdf/002/hehehe.urdf", include_mimic_joints=False)
# actual_robot = RobotModel.from_urdf("/home/rpeterson/repos/python-repo/ihmc-alex-sdk/alex-models/alex_description/urdf/002/hehehe.urdf")
# # actual_robot.name = "actual_robot"
# desired_robot.name = "desired_robot"
# for link in desired_robot.link_list:
#     # link_item: Link = desired_robot.link_list[0]
#     link.set_alpha(0.5)
#
#
# # viewer = ViserViewer()
# # viewer = PyrenderViewer(update_interval=dt)
# viewer = TrimeshSceneViewer(update_interval=dt)
# viewer.add(actual_robot)
# viewer.add(desired_robot)
#
# viewer.show()
# viewer.redraw()
#
# print(desired_robot.joint_names)
# for joint in desired_robot.joint_list:
#     print(joint.name, " ", joint.type)
#
# for link in desired_robot.link_list:
#     print(link.name)
# positions = desired_robot.angle_vector()
# max_positions = desired_robot.joint_max_angles
# min_positions = desired_robot.joint_min_angles
# range = max_positions - min_positions
# print(max_positions, min_positions)
# loop_num = 100
# loop_time = 0
# curr_time = time.perf_counter()
#
# start_time = time.perf_counter_ns()
# while ((time.perf_counter_ns() - start_time) * ns_to_s < 20.0): # or (len(viewer._server.get_clients()) > 0):
#     prev_time = curr_time
#     curr_time = time.perf_counter_ns()
#     loop_time += curr_time - prev_time
#     loop_num += 1
#     viewer.redraw()
#     new_positions = random.uniform(min_positions, max_positions)
#     # print(new_positions)
#     desired_robot.angle_vector(new_positions)
#     # print(len(viewer._server.get_clients()))
#     if (loop_num >= 10000):
#         print ("Avg loop time: ", loop_time/loop_num * ns_to_s)
#         loop_time = 0
#         loop_num = 0
#     elapsed_time = time.perf_counter_ns() - curr_time
#     remaining_time = dt - elapsed_time * ns_to_s
#     # run_slider_panel()
#     if (remaining_time > 0):
#         time.sleep(remaining_time)
#     else:
#         print("missed loop ", remaining_time)