class ControlModeState:
    drive = 0
    stopped = 1
    planning = 2
    execution = 3
    assessing = 4
    automatic = 5
    manual = 6

    descriptions = ['Driving', 'Stopped', 'Planning', 'Execution', 'Assessing', 'Automatic',
                    'Manual']

    def __init__(self, mode):
        self.state = mode

    def __str__(self):
        return self.descriptions[self.state]
