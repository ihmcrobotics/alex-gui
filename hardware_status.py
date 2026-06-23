from dearpygui_helpers import *
from messages import HardwareStatus


class HardwareStatusGUI:
    def __init__(self, plot_length: int=200):
        self._time_count = 0.0
        self._plot_length = plot_length
        self._time = [0.0] * plot_length
        self._power_supply_voltage = [0.0] * plot_length
        self._motor_bus_voltage = [0.0] * plot_length
        self._power_supply_current = [0.0] * plot_length
        self._motor_bus_current = [0.0] * plot_length
        self._power_supply_power = [0.0] * plot_length
        self._motor_bus_power = [0.0] * plot_length

        with dpg.window(label="Hardware Status", tag="hardware_status"):
            dpg.add_text("Robot Status:")
            with dpg.group(horizontal=True):
                self._robot_fault = CheckBox("Robot Fault", False)
                self._motor_fault = CheckBox("Motor Fault", False)
            with dpg.group(horizontal=True):
                self._bus_over_voltage_fault = CheckBox("Bus Over Voltage Fault", False)
                self._bus_over_current_fault = CheckBox("Bus Over Current Fault", False)

            dpg.add_text("Communication Status:")
            with dpg.group(horizontal=True):
                dpg.add_text("Missed Deadlines:")
                self._missed_deadlines = ValueDisplay("Missed Deadlines Fault", initial_value=0)
            self._missed_deadline_fault = CheckBox("Missed Deadline Fault", False)
            with dpg.group(horizontal=True):
                dpg.add_text("Working Counter Mismatches:")
                self._working_counter_mismatches = ValueDisplay("Working Counter Mismatches", initial_value=0)
            self._working_counter_fault = CheckBox("Working Counter Fault", False)

            dpg.add_text("Power Status:")
            with dpg.group(horizontal=True):
                with dpg.plot(label="Voltage", tag="voltage_plot", width=350, height=200):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, tag="voltage_time", no_label=True, no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="voltage", label="Voltage (V)")
                    dpg.set_axis_limits("voltage", ymax=50.0, ymin=0.0)
                    dpg.add_line_series([], [], parent="voltage", label = "Power Supply", tag="power_supply_voltage")
                    dpg.add_line_series([], [], parent="voltage", label = "Motor Bus", tag="motor_bus_voltage")
                with dpg.plot(label="Current", tag="current_plot", width=350, height=200):
                    dpg.add_plot_axis(dpg.mvXAxis, tag="current_time", no_label=True, no_tick_labels=True)
                    dpg.add_plot_axis(dpg.mvYAxis, tag="current", label="Current (A)")
                    dpg.set_axis_limits("current", ymax=10.0, ymin=-10.0)
                    dpg.add_line_series([], [], parent="current", tag="power_supply_current")
                    dpg.add_line_series([], [], parent="current", tag="motor_bus_current")
            # with dpg.plot(label="Power", tag="power_plot", width=400, height=200):
            #     dpg.add_plot_axis(dpg.mvXAxis, tag="power_time", label="Time (s)", no_tick_labels=True)
            #     dpg.add_plot_axis(dpg.mvYAxis, tag="power", label="Power (W)")
            #     dpg.add_line_series([], [], parent="power", tag="power_supply_power")
            #     dpg.add_line_series([], [], parent="power", tag="motor_bus_power")
    
    def set_spacing(self, x_pos: int=0, y_pos: int=0, right_aligned=True):
        dim = dpg.get_item_rect_size("hardware_status")
        if right_aligned:
            dpg.set_item_pos("hardware_status", [x_pos - dim[0], y_pos])
        else:
            dpg.set_item_pos("hardware_status", [x_pos, y_pos])

    def update(self, status: HardwareStatus, robot_time: float):

        self._robot_fault.set(status.robot_fault)
        self._motor_fault.set(status.motor_fault)
        self._missed_deadline_fault.set(status.missed_deadline_fault)
        self._working_counter_fault.set(status.working_counter_fault)
        self._bus_over_voltage_fault.set(status.bus_over_voltage_fault)
        self._bus_over_current_fault.set(status.bus_over_current_fault)

        self._working_counter_mismatches.set(status.working_counter_mismatch_count)
        self._missed_deadlines.set(status.missed_deadlines)

        self._update_plots(status, robot_time)

    def _update_plots(self, status: HardwareStatus, robot_time: float):

        if len(self._time) >= self._plot_length:
            self._time.pop(0)
            self._power_supply_voltage.pop(0)
            self._motor_bus_voltage.pop(0)
            self._power_supply_current.pop(0)
            self._motor_bus_current.pop(0)
            self._power_supply_power.pop(0)
            self._motor_bus_power.pop(0)
        self._time_count += 1

        self._time.append(robot_time)
        self._power_supply_voltage.append(status.power_supply_voltage_volts)
        self._motor_bus_voltage.append(status.motor_bus_voltage_volts)
        self._power_supply_current.append(status.power_supply_current_amps)
        self._motor_bus_current.append(status.motor_bus_current_amps)
        self._power_supply_power.append(status.power_supply_power_watts)
        self._motor_bus_power.append(status.motor_bus_power_watts)

        # self._time.append(self._time_count)
        # self._power_supply_voltage.append(random.random())
        # self._motor_bus_voltage.append(random.random())
        # self._power_supply_current.append(random.random())
        # self._motor_bus_current.append(random.random())
        # self._power_supply_power.append(random.random())
        # self._motor_bus_power.append(random.random())

        x_low = self._time[0]
        x_high = self._time[-1]
        dpg.set_axis_limits("voltage_time", x_low, x_high)
        dpg.set_axis_limits("current_time", x_low, x_high)
        # dpg.set_axis_limits("power_time", x_low, x_high)

        dpg.set_value("power_supply_voltage", [self._time, self._power_supply_voltage])
        dpg.set_value("motor_bus_voltage", [self._time, self._motor_bus_voltage])
        dpg.set_value("power_supply_current", [self._time, self._power_supply_current])
        dpg.set_value("motor_bus_current", [self._time, self._motor_bus_current])
        # dpg.set_value("power_supply_power", [self._time, self._power_supply_power])
        # dpg.set_value("motor_bus_power", [self._time, self._motor_bus_power])



