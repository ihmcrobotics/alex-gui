
class JointSettings:
    def __init__(self, max_torque: float,
                 stiffness: float = -1.0,
                 damping: float = -1.0,
                 max_position_error: float = -1.0,
                 max_velocity_error: float = -1.0):
        self._absolute_max_torque = max_torque
        self._max_torque = max_torque
        self._stiffness = self._max_torque  if stiffness == -1.0 else stiffness
        self._damping = self._max_torque / 10.0 if damping == -1.0 else damping
        self._max_position_error = self._max_torque / self._stiffness if max_position_error == -1.0 else max_position_error
        self._max_velocity_error = self._max_torque / self._damping if max_velocity_error == -1.0 else max_velocity_error

    def update_all_settings(self, desired_stiffness: float, desired_damping: float, desired_max_torque: float,
                            desired_max_position_error: float, desired_max_velocity_error: float) -> None:
        self._stiffness = max(0.0, desired_stiffness)
        self._damping = max(0.0, desired_damping)
        self._max_torque = max(0.0, min(desired_max_torque, self._absolute_max_torque))
        if self.stiffness == 0.0:
            self._max_position_error = desired_max_position_error
        else:
            self._max_position_error = min(desired_max_position_error, self._max_torque / self._stiffness)
        if self.damping == 0.0:
            self._max_velocity_error = desired_max_velocity_error
        else:
            self._max_velocity_error = min(desired_max_velocity_error, self._max_torque / self._damping)


    @property
    def max_torque(self) -> float:
        return self._max_torque

    @property
    def stiffness(self) -> float:
        return self._stiffness

    @property
    def damping(self) -> float:
        return self._damping

    @property
    def max_position_error(self) -> float:
        return self._max_position_error

    @property
    def max_velocity_error(self) -> float:
        return self._max_velocity_error



