class ControlModeState:
    drive = 0
    stopped = 1
    planning = 2
    execution = 3
    assessing = 4
    phase_mission_planning = 5
    phase_mission_execution = 6
    phase_mission_complete = 7 # TODO implement mode in UI
    phase_mission_done = 8
    manual = 9
    auto_drive = 10

    descriptions = ['Driving', 'Stopped',
                    'Planning', 'Execution', 'Assessing',
                    'Planning Phase', 'Execution Phase',
                    'Mission Complete', 'Mission Complete', 'Manual',
                    'Autonomous Drive']

    def __init__(self, mode):
        self.state = mode

    def __str__(self):
        return self.descriptions[self.state]
