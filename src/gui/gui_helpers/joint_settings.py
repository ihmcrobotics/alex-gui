class JointSettings:
    """
    This class holds the torque-specific joint settings for a joint. This includes:
    - stiffness
    - damping
    - max torque
    - max position error
    - max velocity error

    """
    def __init__(self, max_torque: float,
                 stiffness: float = -1.0,
                 damping: float = -1.0,
                 max_position_error: float = -1.0,
                 max_velocity_error: float = -1.0):
        """
        Initialize the joint settings
        :param max_torque: Max torque of the actuator for the joint
        :param stiffness: Initial impedance control stiffness. If -1.0, stiffness = max torque. Defaults to -1.0.
        :param damping: Initial impedance control damping. If -1.0, damping = max torque / 10.0. Defaults to -1.0.
        :param max_position_error: Max position error for impedance control.
        If -1.0, max error = max torque / stiffness. Defaults to -1.0.
        :param max_velocity_error: Max velocity error for impedance control.
        If -1.0, max error = max torque / damping. Defaults to -1.0.
        """
        self._absolute_max_torque = max_torque
        self._max_torque = max_torque
        self._stiffness = self._max_torque  if stiffness == -1.0 else stiffness
        self._damping = self._max_torque / 10.0 if damping == -1.0 else damping
        self._max_position_error = self._max_torque / self._stiffness if max_position_error == -1.0 else max_position_error
        self._max_velocity_error = self._max_torque / self._damping if max_velocity_error == -1.0 else max_velocity_error

    def update_all_settings(self, desired_stiffness: float, desired_damping: float, desired_max_torque: float,
                            desired_max_position_error: float, desired_max_velocity_error: float) -> None:
        """
        Update all the settings, including applying limits based on the max torque, stiffness, and damping
        :param desired_stiffness: New stiffness
        :param desired_damping: New damping
        :param desired_max_torque: New max torque. Saturates at the actuator max torque
        :param desired_max_position_error: New max position error. Saturates at max torque / stiffness
        :param desired_max_velocity_error: New max velocity error. Saturates at max torque / damping
        """
        # Make sure new values are non-zero
        self._stiffness = max(0.0, desired_stiffness)
        self._damping = max(0.0, desired_damping)
        self._max_torque = max(0.0, min(desired_max_torque, self._absolute_max_torque))

        # Make sure new max errors fit with the new impedance control settings and max torque
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



