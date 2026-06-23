import time
from threading import Lock
from typing import List, Dict

from skrobot.model import RobotModel
from skrobot.viewers import PyrenderViewer


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
                command = data["alex_command"]
                for i in range(len(self.ghost_robot.joint_list)):
                    desired = command.joint_commands[i]
                    self.ghost_robot.joint_list[i].joint_angle(desired.q_des)

                joint_states = data["alex_state"].joint_states

                if len(joint_states) > 0:
                    for joint_state in joint_states:
                        joint_name = joint_state.joint_name
                        self._joints[joint_name].joint_angle(joint_state.q)
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

    def get_joint_names(self) -> List[str]:
        return self.robot.joint_names

    def get_robot_model(self) -> RobotModel:
        return self.robot