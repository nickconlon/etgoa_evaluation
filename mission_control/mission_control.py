class MissionControl:
    OKAY = 'okay'
    GPS = 'gps'
    BATTERY = 'battery'
    OBSTACLE = 'obstacle'
    EVERYTHING = 'everything'

    def __init__(self):
        self.help_requests = {"": None,
                              "I need help diagnosing everything": self.EVERYTHING,
                              "I need help diagnosing robot position issues": self.GPS,
                              "I need help diagnosing robot battery issues": self.BATTERY,
                              "I need help with an obstacle blocking the robot": self.OBSTACLE}

        self.help_responses = {
            self.OKAY: '{} looks fine to us',
            self.GPS: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, set the GPS to frequency 34. Then instruct the robot continue.',
            self.BATTERY: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, switch to backup battery 3. Then set power to 75. Then instruct the robot continue.',
            self.OBSTACLE: 'Please follow this procedure:\n\nInstruct the robot to stop. Next, switch to manual control and drive the robot around the obstacle and to the next waypoint. Then return control to the robot'}

        self.current_issue = self.BATTERY

    def get_response(self, issue):
        if self.current_issue == issue:
            return self.help_responses[issue]
        else:
            return self.help_responses[self.OKAY].format(issue)