import math
import tkinter as tk
from collections import deque
import random
from tkinter import DoubleVar
from typing import List, Dict

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from joint_control_gui import JointControlGUI


class AlexDataGUI:
    def __init__(self, root, arm_control: JointControlGUI):
        self.root = root
        self.plotter_window = tk.Toplevel(root)
        self._joint_names = arm_control.joint_names
        self._measured_positions = arm_control.measured_joint_positions
        self._desired_positions = arm_control.desired_joint_positions

        # 1. Initialize data buffers (keeps the last 100 points)
        self.max_points = 10
        self.x_data = list(range(self.max_points))
        self.measured_joint_data = {joint_name: deque([0.0] * self.max_points, maxlen=self.max_points) for joint_name in self._joint_names}
        self.desired_joint_data = {joint_name: deque([0.0] * self.max_points, maxlen=self.max_points) for joint_name in self._joint_names}
        self.y_data = deque([0.0] * self.max_points, maxlen=self.max_points)

        # 2. Configure Matplotlib Figure and Axes
        self.fig = Figure(figsize=(5, 10), dpi=100)
        n_cols = 2
        n_rows = int(len(self._joint_names) / 2)
        self._axes = {}
        self._measured_lines = {}
        self._desired_lines = {}
        count = 1
        for joint_name in self._joint_names:
            self._axes[joint_name] = self.fig.add_subplot(n_rows, n_cols, count)
            self._axes[joint_name].set_title(joint_name + " position")
            self._axes[joint_name].set_ylim(-math.pi, math.pi)

            self._measured_lines[joint_name], = self._axes[joint_name].plot(self.x_data, list(self.measured_joint_data[joint_name]), color="blue", animated=True)
            self._desired_lines[joint_name], = self._axes[joint_name].plot(self.x_data,
                                                                            list(self.desired_joint_data[joint_name]),
                                                                            color="red", animated=True)
            count += 1
        # self.ax = self.fig.add_subplot(111)
        # self.ax.set_title("Live Sensor Readings")
        # self.ax.set_ylim(-10, 10)  # Fixed Y-axis keeps the plot from jumping
        #
        # # Create an empty line object that we will update dynamically
        # self.line, = self.ax.plot(self.x_data, list(self.y_data), color="blue")

        # 3. Embed Matplotlib Figure into the Tkinter Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plotter_window)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.fig.canvas.draw()
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)

        # 4. Start the real-time update loop
        self.update_plot()

    def update_plot(self):
        self.fig.canvas.restore_region(self.bg)
        # Simulate receiving a new real-time data point
        for joint_name in self._joint_names:
            measured_joint_data = self.measured_joint_data[joint_name]
            desired_joint_data = self.desired_joint_data[joint_name]
            # new_value = random.uniform(-math.pi, math.pi)
            # measured_joint_data.append(new_value)
            # desired_joint_data.append(-new_value)
            measured_joint_data.append(self._measured_positions[joint_name])
            desired_joint_data.append(self._desired_positions[joint_name].get())

            # Update the line data rather than clearing/re-plotting the whole axis
            self._measured_lines[joint_name].set_ydata(list(measured_joint_data))
            self._desired_lines[joint_name].set_ydata(list(desired_joint_data))
            self._axes[joint_name].draw_artist(self._measured_lines[joint_name])
            self._axes[joint_name].draw_artist(self._desired_lines[joint_name])


        # Dynamic Y-axis adjustment (optional, uncomment if your scale fluctuates)
        # self.ax.set_ylim(min(self.y_data) - 1, max(self.y_data) + 1)

        # Redraw the canvas to show changes
        self.fig.canvas.blit(self.fig.bbox)
        self.fig.canvas.flush_events()

        # Schedule the next update in 100 milliseconds
        self.root.after(100, self.update_plot)