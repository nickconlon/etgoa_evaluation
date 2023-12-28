import traceback
import pandas as pd


class DataRecorder:
    def __init__(self, condition):
        """
        position (latitude, longitude, altitude)
        heading (degrees from N)
        velocity (m/s)
        control mode (driving | stopped)
        mission mode (planning | execution)
        participant help request (text | 0)
        mission control response (text | 0)
        time
        """
        self.fname = './data/test_recorder.csv'
        self.header = ['latitude', 'longitude', 'altitude', 'heading', 'velocity',
                       'battery number', 'battery remaining', 'power value', 'gps frequency',
                       'control mode', 'mission mode',
                       'help request', 'help response', 'timestamp', 'condition']
        pd.DataFrame(columns=self.header).to_csv(self.fname, index=False)
        self.condition = condition
        self.data = []

    def add_row(self, latitude, longitude, altitude,
                heading, velocity,
                control_mode, mission_mode,
                battery_number, battery_remaining,
                power_value, gps_value,
                participant_help_request,
                mission_control_response, mission_time):
        try:
            row = [latitude, longitude, altitude, heading, velocity,
                   control_mode, mission_mode,
                   battery_number, battery_remaining, power_value, gps_value,
                   participant_help_request,
                   mission_control_response,
                   mission_time, self.condition]
            self.data.append(row)
            d = {c: [d] for c, d in zip(self.header, row)}
            pd.DataFrame(d).to_csv(self.fname, mode='a', index=False, header=False)
        except Exception as e:
            traceback.print_exc()
