import numpy as np


class Battery:
    def __init__(self):
        self.battery_level = 100
        self.eff = 1
        self.battery_numer = 0

    def swap_battery(self, number):
        self.battery_level = 100
        self.battery_numer = number

    def get_number(self):
        return self.battery_numer

    def get_level(self):
        return self.battery_level

    def decrement(self, T):
        self.battery_level = np.maximum(0, self.battery_level - T * self.eff)
