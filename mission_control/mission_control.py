import numpy as np


class MissionControl:
    OKAY = 'okay'
    GPS = 'gps'
    BATTERY = 'battery'
    OBSTACLE = 'obstacle'
    EVERYTHING = 'everything'
    GENERAL = 'general'
    POIs = ['A', 'B', 'C', 'D']

    def __init__(self, base_gps, base_power, base_battery, gps_max, power_max, battery_max):
        self.help_responses = {
            self.OKAY: '{} looks fine to us',
            self.GPS: 'Please follow this procedure:\n\nInstruct the robot to stop.Set the GPS {}. The POWER can remain at {}. The battery can remain at {}.',
            self.BATTERY: 'Please follow this procedure:\n\nInstruct the robot to stop. The GPS can remain at {}. Set the POWER to {}. Set the battery to {}.',
            self.OBSTACLE: 'Please follow this procedure:\n\nInstruct the robot to stop. The GPS can remain at {}. Set the POWER to {}. Set the battery to {}.',
            self.GENERAL: 'Please follow this procedure:\n\nInstruct the robot to stop. Set the Battery to {}. Set the POWER to {}, and set the GPS frequency to {}.'}

        self.current_issue = self.BATTERY
        self.battery_max = battery_max
        self.gps_max = gps_max
        self.power_max = power_max
        self.battery = base_battery
        self.power = base_power
        self.gps = base_gps
        self.mission_pois = []

    def set_mission_pois(self, pois):
        self.mission_pois = []
        for p in pois:
            self.mission_pois.append(p)

    def send_mission(self):
        text = 'Please choose a POI for the robot explore.\n\nAvailable POIs for this mission: '+', '.join(self.mission_pois)
        return text

    def get_response(self, anomaly):
        if anomaly:
            self.gps = np.random.randint(0, self.gps_max)
            self.battery = np.random.randint(1, self.battery_max)
            self.power = np.random.randint(5, self.power_max)
            return self.help_responses[self.GENERAL].format(self.battery, self.power, self.gps)
        else:
            return 'Everything looks fine from here!'

    def check_strategy(self, power, gps, battery):
        if self.battery == battery and self.gps == gps and self.power == power:
            return "Looks like that fixed the anomaly. Replan to the POI."
        else:
            return ""
