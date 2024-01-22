import traceback
import pandas as pd


class DataRecorderBase:
    def __init__(self, path, header):
        self.path = path
        self.header = header
        self.data = []

    def write_header(self):
        try:
            pd.DataFrame(columns=self.header).to_csv(self.path, index=False)
        except Exception as e:
            traceback.print_exc()

    def record(self, row):
        self.data.append(row)
        d = {c: [d] for c, d in zip(self.header, row)}
        pd.DataFrame(d).to_csv(self.path, mode='a', index=False, header=False)


class ConcurrentTaskRecorder(DataRecorderBase):
    def __init__(self, path):
        header = ['time stamp', 'correct', 'decision time', 'mineral']
        DataRecorderBase.__init__(self, path, header)
        self.write_header()

    def add_row(self, timestamp, correct, decition_time, mineral):
        row = [timestamp, correct, decition_time, mineral]
        self.record(row)


class SurveyRecorder:
    def __init__(self, trust_path, usability_path):
        if trust_path is not None:
            header = ["Q{}".format(x+1) for x in range(14)] + ['score'] + ['time stamp']
            self.trust_recorder = DataRecorderBase(trust_path, header)
            self.trust_recorder.write_header()

        if usability_path is not None:
            header = ["Q{}".format(x + 1) for x in range(10)] + ['score'] + ['time stamp']
            self.usability_recorder = DataRecorderBase(usability_path, header)
            self.usability_recorder.write_header()

    def record_trust(self, responses, score, timestamp):
        row = [*responses, score, timestamp]
        self.trust_recorder.record(row)

    def record_usability(self, responses, score, timestamp):
        row = [*responses, score, timestamp]
        self.usability_recorder.record(row)



class PrimaryTaskRecorder(DataRecorderBase):
    def __init__(self, condition, fname):
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
        header = ['latitude', 'longitude', 'altitude',
                  'heading', 'velocity',
                  'control mode', 'mission mode',
                  'battery number', 'battery remaining',
                  'power value', 'gps frequency',
                  'help request', 'help response',
                  'goa', 'mqa',
                  'timestamp',
                  'condition']
        DataRecorderBase.__init__(self, fname, header)
        self.write_header()
        self.condition = condition

    def add_row(self, latitude, longitude, altitude,
                heading, velocity,
                control_mode, mission_mode,
                battery_number, battery_remaining,
                power_value, gps_value,
                participant_help_request, mission_control_response,
                goa, mqa,
                mission_time):
        try:
            row = [latitude, longitude, altitude,
                   heading, velocity,
                   control_mode, mission_mode,
                   battery_number, battery_remaining,
                   power_value, gps_value,
                   participant_help_request,
                   mission_control_response,
                   goa, mqa,
                   mission_time,
                   self.condition]
            self.record(row)
        except Exception as e:
            traceback.print_exc()
