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
        self.backup_batts_used = 0

    def set_mission_pois(self, pois):
        self.mission_pois = []
        for p in pois:
            self.mission_pois.append(p)

    def send_mission(self):
        text = 'Available POIs for this mission:\n\n'+', '.join(self.mission_pois)
        return text

    def get_response(self, anomaly, anomaly_type):
        if anomaly:
            self.gps = np.random.randint(0, self.gps_max) if 'h' in anomaly_type else self.gps
            self.battery = np.random.randint(1, self.battery_max) if 'b' in anomaly_type else self.battery
            self.backup_batts_used = self.backup_batts_used + 1 if 'b' in anomaly_type else self.backup_batts_used
            self.power = np.random.randint(5, self.power_max) if 'h' in anomaly_type or 'b' in anomaly_type else self.power
            return self.help_responses[self.GENERAL].format(self.battery, self.gps, self.power)
        else:
            return 'Everything looks fine from here!'

    def check_strategy(self, power, gps, battery):
        if self.battery == battery and self.gps == gps and self.power == power:
            return "Looks like that fixed the anomaly!\nPlease replan to the POI."
        else:
            return ""
