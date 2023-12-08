
class Battery:
    def __init__(self):
        self.battery_remaining = 100

    def decrement_and_return(self):
        self.battery_remaining -= 1
        return self.battery_remaining
