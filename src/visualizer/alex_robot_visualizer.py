import time
from threading import Lock
from typing import List, Dict

from skrobot.model import RobotModel
from skrobot.viewers import PyrenderViewer


class AlexVisualizer:
    """
    This class creates a visualizer for Alex using PyrenderViewer from Scikit-Robot.
    In the visualizer, there are two robots: the main robot and the ghost robot.
    The main robot is used to show the measured joint positions of Alex, while the ghost robot is a semi-transparent
    copy of the main robot that shows the desired positions for each joint on Alex.
    """

    def __init__(self, urdf_path: str, ghost_path: str | None = None, frequency: float = 100.0):
        """
        Initialize the visualizer class.

        :param urdf_path: Path to the URDF description of the robot
        :param ghost_path: Path to the ghost description of the robot if there is one. If None, it uses the urdf_path.
        Default is None.
        :param frequency: Update frequency of the visualizer in Hz
        """
        self.robot = RobotModel.from_urdf(urdf_path)
        self.ghost_robot = RobotModel.from_urdf(ghost_path if ghost_path is not None else urdf_path)
        self.ghost_robot.name = "ghost"
        self.frequency = frequency
        self.dt = 1.0 / self.frequency
        self._joints = {joint.name: joint for joint in self.robot.joint_list}
        self._viewer = None
        self._curr_time = time.perf_counter_ns()

        # Make the ghost robot semi-transparent
        for link in self.ghost_robot.link_list:
            link.set_alpha(0.5)
        self.initialized = False

    def initialize(self) -> None:
        """
        Initialize and display the visualizer window.
        """

        self._viewer = PyrenderViewer(update_interval=self.dt)

        # Add the robots to the visualizer
        self._viewer.add(self.robot)
        self._viewer.add(self.ghost_robot)

        self._viewer.show()
        self._viewer.redraw()
        self.initialized = True

    def update(self, lock: Lock, data: Dict) -> None:
        """
        Update the visualizer with new data

        :param lock: Threading lock for the shared memory
        :param data: Dictionary of data from shared memory
        """

        if not self.initialized:
            self.initialize()
        else:
            with lock:
                # Update the ghost robot with the desired joint positions
                command = data["alex_command"]
                for i in range(len(self.ghost_robot.joint_list)):
                    desired = command.joint_commands[i]
                    self.ghost_robot.joint_list[i].joint_angle(desired.q_des)

                # Update the main robot with the measured joint positions
                joint_states = data["alex_state"].joint_states
                if len(joint_states) > 0:
                    for joint_state in joint_states:
                        joint_name = joint_state.joint_name
                        self._joints[joint_name].joint_angle(joint_state.q)
            self._viewer.redraw()

    def run_visualizer(self, lock: Lock, data: Dict):
        """
        Run the visualizer.

        :param lock: Threading lock for the shared memory
        :param data: Dictionary of data from shared memory
        """
        if not self.initialized:
            self.initialize()

        while True:
            self._curr_time = time.perf_counter_ns()
            self.update(lock, data)

            # Keep the loop time deterministic
            elapsed_time = time.perf_counter_ns() - self._curr_time
            remaining_time = self.dt - elapsed_time * 1.0e-9
            if remaining_time > 0:
                time.sleep(remaining_time)

    def get_robot_model(self) -> RobotModel:
        """
        :return: The robot model of the main robot
        """
        return self.robot
