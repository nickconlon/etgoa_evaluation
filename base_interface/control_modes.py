class ControlModeState:
    driving = 0
    stopped = 1
    #planning = 2
    execution = 3
    assessing = 4
    phase_mission_planning = 5
    phase_mission_execution = 6
    phase_mission_complete = 7
    phase_mission_done = 8
    manual_drive = 9
    auto_drive = 10

    planning = 11
    planning_assessing = 12
    executing = 13
    executing_manual = 14
    executing_manual_driving = 15
    executing_manual_stopped_assessing = 16
    executing_auto_stopped = 17
    executing_auto_driving = 18
    executing_auto_stopped_assessing = 19
    anomaly_found = 20
    completed = 21

    descriptions = ['Driving', 'Stopped',
                    'Planning', 'Execution', 'Assessing',
                    'Planning Phase', 'Execution Phase',
                    'Mission Complete', 'Mission Complete', 'Manual',
                    'Autonomous Drive',

                    'Planning',
                    'Planning|Assessing',
                    'Executing',
                    'Executing|Manual',
                    'Executing|Manual|Driving',
                    'Executing|Manual|Stopped|Assessing',
                    'Executing|Auto|Stopped',
                    'Executing|Auto|Driving',
                    'Executing|Auto|Stopped|Assessing',
                    'Anomaly',
                    'Completed']

    def __init__(self, mode):
        self.state = mode

    def __str__(self):
        return self.descriptions[self.state]
