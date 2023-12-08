import traceback


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
                       'control mode', 'mission mode', 'loa',
                       'help request', 'help response', 'timestamp']
        with open('test.csv', 'w') as f:
            f.write(','.join(self.header) + '\n')

        self.data = []

    def add_row(self, latitude, longitude, altitude,
                heading, velocity,
                control_mode, mission_mode, loa,
                battery_number, battery_remaining,
                power_value, gps_value,
                participant_help_request,
                mission_control_response, mission_time):
        try:
            row = (latitude, longitude, altitude, heading, velocity,
                   control_mode, mission_mode, loa,
                   battery_number, battery_remaining, power_value, gps_value,
                   participant_help_request,
                   mission_control_response,
                   mission_time)
            self.data.append(row)
            with open('test.csv', 'a') as f:
                f.write(','.join([str(r) for r in row]) + '\n')
        except Exception as e:
            traceback.print_exc()
