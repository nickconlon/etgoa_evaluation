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
        header = ['time stamp', 'actual_x', 'actual_y', 'actual_std', 'x', 'y', 'decision time', 'mineral']
        DataRecorderBase.__init__(self, path, header)
        self.write_header()

    def add_row(self, timestamp, actual_x, actual_y, actual_std, x, y, decition_time, mineral):
        row = [timestamp, actual_x, actual_y, actual_std, x, y, decition_time, mineral]
        self.record(row)


class TrustRecorder:
    def __init__(self, path):
        header = ["Q{}".format(x+1) for x in range(14)] + ['score'] + ['time stamp']
        self.trust_recorder = DataRecorderBase(path, header)
        self.trust_recorder.write_header()

    def record(self, responses, timestamp):
        row = [*responses, timestamp]
        self.trust_recorder.record(row)


class UsabilityRecorder:
    def __init__(self, path):
        header = ["Q{}".format(x + 1) for x in range(10)] + ['score'] + ['time stamp']
        self.usability_recorder = DataRecorderBase(path, header)
        self.usability_recorder.write_header()

    def record(self, responses, timestamp):
        row = [*responses, timestamp]
        self.usability_recorder.record(row)

class QuestionsRecorder:
    def __init__(self, path):
        self.recorder = DataRecorderBase(path, '')

    def record(self, responses, timestamp):
        row = [*responses, timestamp]
        self.recorder.record(row)


class DemographicsRecorder:
    def __init__(self, path):
        header = ['age', 'gender', 'robotics experience', 'gaming experience', 'time stamp']
        self.usability_recorder = DataRecorderBase(path, header)
        self.usability_recorder.write_header()

    def record(self, responses, timestamp):
        row = [*responses, timestamp]
        self.usability_recorder.record(row)


class PrimaryTaskRecorder(DataRecorderBase):
    def __init__(self, condition, configuration, fname):
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
                  'state',
                  'battery number', 'battery remaining',
                  'power value', 'gps frequency',
                  'anomaly state',
                  'mission control text',
                  'goa', 'mqa', 'mission outcomes', 'resources found',
                  'timestamp',
                  'condition', 'configuration']
        DataRecorderBase.__init__(self, fname, header)
        self.write_header()
        self.condition = condition
        self.configuration = configuration

    def add_row(self, latitude, longitude, altitude,
                heading, velocity,
                state,
                battery_number, battery_remaining,
                power_value, gps_value,
                experiencing_anomaly,
                mission_control_text,
                goa, mqa, mission_outcomes, resources_found,
                mission_time):
        try:
            row = [latitude, longitude, altitude,
                   heading, velocity,
                   state,
                   battery_number, battery_remaining,
                   power_value, gps_value,
                   experiencing_anomaly,
                   mission_control_text,
                   goa, mqa, mission_outcomes, resources_found,
                   mission_time,
                   self.condition, self.configuration]
            self.record(row)
        except Exception as e:
            traceback.print_exc()
