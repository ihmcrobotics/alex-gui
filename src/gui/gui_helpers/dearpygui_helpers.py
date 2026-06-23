from dataclasses import dataclass
from typing import Any, Dict

import dearpygui.dearpygui as dpg


def update_value(sender, app_data, user_data):
    user_data.set(app_data)

@dataclass
class Value:
    value: int | float | bool

    def set(self, value: Any):
        self.value = value

class FloatSlider:
    def __init__(self, name: str, suffix: str="_slider", min_value: float=0.0, max_value: float=1.0,
                 initial_value: float=0.0, use_name_as_label: bool=True, width: int=300):
        self._name = name
        self._tag = name+suffix
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
    def __init__(self, name: str, initial_value: bool=False, use_name_as_label: bool=True):
        self._name = name
        self._tag = name+"_checkbox"
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
    def __init__(self, name: str):
        self._name = name
        self._tag = name+"_button"
        self._value = Value(False)
        dpg.add_button(label=name, tag=self._tag, user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: bool):
        self._value.set(value)
        dpg.set_value(self._tag, value)

class ValueDisplay:
    def __init__(self, name: str, suffix: str = "_display", initial_value: int | float | bool = 0):
        self._name = name
        self._tag = name+suffix
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

class ComboBox:
    def __init__(self, name: str, combo_dict: Dict[str, Any], suffix = "_combo_box"):
        self._name = name
        self._combo_dict = combo_dict
        combo_list = list(combo_dict)
        self._tag = name+suffix
        self._value = Value(combo_dict[combo_list[0]])
        dpg.add_combo(label=name, tag=self._tag, items=combo_list, user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: str):
        self._value.set(self._combo_dict[value])
        dpg.set_value(self._tag, value)
