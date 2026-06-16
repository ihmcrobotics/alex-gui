from dataclasses import dataclass

import dearpygui.dearpygui as dpg

def update_value(sender, app_data, user_data):
    user_data.set(app_data)

@dataclass
class Float:
    value: float

    def set(self, value: float):
        self.value = value

@dataclass
class Boolean:
    value: bool

    def set(self, value: bool):
        self.value = value

class FloatSlider:
    def __init__(self, name: str, min_value: float=0.0, max_value: float=1.0, initial_value: float=0.0, use_name_as_label: bool=True):
        self._name = name
        self._tag = name+"_slider"
        self._min_value = min_value
        self._max_value = max_value
        self._initial_value = initial_value
        self._value = Float(initial_value)
        if use_name_as_label:
            dpg.add_slider_float(label=name, tag=self._tag,
                                 min_value=min_value, max_value=max_value, default_value=initial_value,
                                 user_data=self._value, callback=update_value)
        else:
            dpg.add_slider_float(tag=self._tag,
                                 min_value=min_value, max_value=max_value, default_value=initial_value,
                                 user_data=self._value, callback=update_value)
    @property
    def value(self):
        return self._value.value

    def set(self, value: float):
        dpg.set_value(self._tag, value)

class CheckBox:
    def __init__(self, name: str, initial_value: bool=False):
        self._name = name
        self._tag = name+"_checkbox"
        self._value = Boolean(initial_value)
        dpg.add_checkbox(label=name, tag=self._tag, default_value=initial_value,
                         user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: float):
        dpg.set_value(self._tag, value)

class Button:
    def __init__(self, name: str):
        self._name = name
        self._tag = name+"_button"
        self._value = Boolean(False)
        dpg.add_button(label=name, tag=self._tag, user_data=self._value, callback=update_value)

    @property
    def value(self):
        return self._value.value

    def set(self, value: float):
        dpg.set_value(self._tag, value)

class FloatDisplay:
    def __init__(self, name: str, initial_value: float=0.0):
        self._name = name
        self._tag = name+"_display"
        self._value = Float(0.0)
        dpg.add_text(label=name + ": " + f"{initial_value:.3f}", tag=self._tag)

    @property
    def value(self):
        return self._value.value

    def set(self, value: float):
        self._value.set(value)
        dpg.configure_item(self._tag, label=self._name + ": " + f"{value:.3f}" )