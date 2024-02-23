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
        self.tmp_anomaly_type = ''
        self.tmp_battery_level = 100

    def set_mission_pois(self, pois):
        self.mission_pois = []
        for p in pois:
            self.mission_pois.append(p)

    def send_mission(self):
        text = 'Available POIs for this mission:\n\n'+', '.join(self.mission_pois)
        return text

    def get_response(self, anomaly, anomaly_type, batt_level):
        if anomaly:
            self.tmp_anomaly_type = anomaly_type
            self.tmp_battery_level = batt_level
            self.gps = np.random.randint(0, self.gps_max) if 'h' in anomaly_type else self.gps
            if self.tmp_battery_level < 75:
                self.battery = np.random.randint(1, self.battery_max) if 'b' in anomaly_type else self.battery

            self.power = np.random.randint(5, self.power_max) if 'h' in anomaly_type or 'b' in anomaly_type else self.power
            return self.help_responses[self.GENERAL].format(self.battery, self.gps, self.power)
        else:
            return 'Everything looks fine from here!\n\nIf something looks wrong,\ntry using Make Plan'

    def check_strategy(self, power, gps, battery):
        if self.battery == battery and self.gps == gps and self.power == power:
            if self.tmp_battery_level < 75:
                self.backup_batts_used = self.backup_batts_used + 1 if 'b' in self.tmp_anomaly_type else self.backup_batts_used
            self.tmp_anomaly_type = ''
            self.tmp_battery_level = 100
            return "Looks like that fixed the anomaly!"
        else:
            return ""
