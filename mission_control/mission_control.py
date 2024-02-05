import numpy as np


class MissionControl:
    OKAY = 'okay'
    GENERAL = 'general'

    def __init__(self, base_gps, base_power, base_battery, gps_max, power_max, battery_max):
        self.help_responses = {
            self.OKAY: '{} looks fine to us',
            self.GENERAL: 'Please follow this procedure:\n'
                          '\nSet the Backup Battery to {}.'
                          '\nSet the GPS Frequency to {}.'
                          '\nFinally, set the Motor Power {}.'}
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
        text = 'Available POIs for this mission:\n\n'+', '.join(self.mission_pois)
        return text

    def get_response(self, anomaly):
        if anomaly:
            self.gps = np.random.randint(0, self.gps_max)
            self.battery = np.random.randint(1, self.battery_max)
            self.power = np.random.randint(5, self.power_max)
            return self.help_responses[self.GENERAL].format(self.battery, self.gps, self.power)
        else:
            return 'Everything looks fine from here!'

    def check_strategy(self, power, gps, battery):
        if self.battery == battery and self.gps == gps and self.power == power:
            return "Looks like that fixed the anomaly!\nPlea replan to the POI."
        else:
            return ""
