"""
This file contains helper classes and methods for using sliders, buttons, checkboxes, and displays in dearpygui
"""

from dataclasses import dataclass
from typing import Any, Dict

import dearpygui.dearpygui as dpg


def update_value(sender, app_data, user_data):
    """
    Update the value of the variable tied to a dearpygui item

    :param sender: The dearpygui item
    :param app_data: the value from the dearpygui item
    :param user_data: The variable tied to the dearpygui item
    """
    user_data.set(app_data)


@dataclass
class Value:
    """
    This class allows for the storing and updating of a generic primitive value.
    This makes primitives usable within the dearpygui callback system
    """
    value: int | float | bool

    def set(self, value: Any):
        self.value = value


class FloatSlider:
    """
    Class representation of a dearpygui slider to tie a variable to the slider more easily
    """

    def __init__(self, name: str, suffix: str = "_slider", min_value: float = 0.0, max_value: float = 1.0,
                 initial_value: float = 0.0, use_name_as_label: bool = True, width: int = 300):
        """
        Initialize the slider and variable

        :param name: Name of the slider. Will be used as the label for the slider
        :param suffix: Tag suffix to be used with the name to make the tag
        :param min_value: Minimum value of the slider
        :param max_value: Maximum value of the slider
        :param initial_value: Initial value of the slider
        :param use_name_as_label: If true, include the label for the slider
        :param width: Width of the slider
        """
        self._name = name
        self._tag = name + suffix
        self._min_value = min_value
        self._max_value = max_value
        self._initial_value = initial_value
        self._value = Value(initial_value)
        if use_name_as_label:
            dpg.add_slider_float(label=name, tag=self._tag,
                                 min_value=min_value, max_value=max_value, default_value=initial_value,
                                 user_data=self._value, callback=update_value, width=width)
        else:
            dpg.add_slider_float(tag=self._tag,
                                 min_value=min_value, max_value=max_value, default_value=initial_value,
                                 user_data=self._value, callback=update_value, width=width)

    @property
    def value(self):
        return self._value.value

    def set(self, value: float):
        self._value.set(value)
        dpg.set_value(self._tag, value)


class CheckBox:
    """
    Class representation of a dearpygui checkbox to tie a boolean to the checkbox more easily
    """

    def __init__(self, name: str, initial_value: bool = False, use_name_as_label: bool = True):
        """
        Initialize the checkbox

        :param name: Name of the checkbox
        :param initial_value: Initial value of the checkbox. Defaults to False
        :param use_name_as_label: Use the name of the checkbox as label. Defaults to True
        """
        self._name = name
        self._tag = name + "_checkbox"
        self._value = Value(initial_value)
        if use_name_as_label:
            dpg.add_checkbox(label=name, tag=self._tag, default_value=initial_value,
                             user_data=self._value, callback=update_value)
        else:
            dpg.add_checkbox(tag=self._tag, default_value=initial_value,
                             user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: bool):
        self._value.set(value)
        dpg.set_value(self._tag, value)


class Button:
    """
    Class representation of a dearpygui button to tie a boolean to the button more easily
    """

    def __init__(self, name: str):
        """
        Initialize the button
        :param name: Mame of the button
        """
        self._name = name
        self._tag = name + "_button"
        self._value = Value(False)
        dpg.add_button(label=name, tag=self._tag, user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: bool):
        self._value.set(value)
        dpg.set_value(self._tag, value)


class ValueDisplay:
    """
    Class representation of a dearpygui text to tie a value to a text for disply
    """

    def __init__(self, name: str, suffix: str = "_display", initial_value: int | float | bool = 0):
        """
        Initialize the value display

        :param name: Name of the value display
        :param suffix: Suffix to be used with the name to make the tag
        :param initial_value: Initial value of the display
        """
        self._name = name
        self._tag = name + suffix
        self._value = Value(initial_value)
        dpg.add_text(default_value="0", tag=self._tag)

    @property
    def value(self):
        return self._value.value

    def set(self, value: float | int | bool):
        self._value.set(value)
        if type(value) is float:
            dpg.set_value(self._tag, str(round(self._value.value, 3)))
        else:
            dpg.set_value(self._tag, str(self._value.value))
