import traceback
import pandas as pd


class DataRecorder:
    def __init__(self):
        """
        position (latitude, longitude, altitude)
        heading (degrees from N)
        velocity (m/s)
        control mode (driving | stopped)
        level of autonomy (manual | automatic)
        mission mode (planning | execution)
        participant help request (text | 0)
        mission control response (text | 0)
        time
        """
        self.header = ['latitude', 'longitude', 'altitude', 'heading', 'velocity',
                       'battery number', 'battery remaining', 'power value', 'gps frequency',
                       'control mode', 'mission mode',
                       'help request', 'help response', 'timestamp']
        pd.DataFrame(columns=self.header).to_csv('test_recorder.csv', index=False)

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
                   mission_time]
            self.data.append(row)
            d = {c: [d] for c, d in zip(self.header, row)}
            pd.DataFrame(d).to_csv('test_recorder.csv', mode='a', index=False, header=False)
        except Exception as e:
            traceback.print_exc()


rec = DataRecorder()
rec.add_row(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, )
rec.add_row(0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, )
