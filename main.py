from pymavlink import mavutil
import time
from datetime import datetime, timezone
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import math
import struct

master = mavutil.mavlink_connection('/dev/ttyACM1')
# master = mavutil.mavlink_connection('/dev/tty.usbmodem1101')
# master = mavutil.mavlink_connection('udp:192.168.0.98:14561')
master.wait_heartbeat()
time.sleep(2)

bucket = "createv"
org = "my-org"
client = InfluxDBClient(url="http://localhost:8086", token="my-token", org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)


class MavlinkData:
    def __init__(self, mavlink_data):
        if mavlink_data is None:
            self.data = None
            return None

        self.type_capture = ['AHRS3', 'AIRSPEED_AUTOCAL', 'AOA_SSA', 'ATTITUDE', 'BATTERY2', 'EKF_STATUS_REPORT',
                             'RC_CHANNELS', 'SERVO_OUTPUT_RAW', 'VFR_HUD', 'WIND', 'POWER_STATUS',
                             'NAV_CONTROLLER_OUTPUT', 'HEARTBEAT', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'GPS2_RAW',
                             'BATTERY_STATUS', 'SYS_STATUS', 'SYSTEM_TIME', 'RAW_IMU', 'PARAM_VALUE']


        # self.type_special = ['NAMED_VALUE_FLOAT']

        if mavlink_data._type in self.type_capture:
            self.data = mavlink_data
        else:
            self.data = None

    def MavlinkData(self):
        return self.data

    def _process_battery_status(self):
        data_dict = {}

        data_dict["battery_1_voltage"] = (self.data.voltages[0] >> 8 & 0x00FF ) * 0.01 + 2.2
        data_dict["battery_2_voltage"] = (self.data.voltages[1] >> 8 & 0x00FF ) * 0.01 + 2.2
        data_dict["battery_3_voltage"] = (self.data.voltages[2] >> 8 & 0x00FF ) * 0.01 + 2.2
        data_dict["battery_4_voltage"] = (self.data.voltages[0] & 0x00FF ) * 0.01 + 2.2
        data_dict["battery_5_voltage"] = (self.data.voltages[1] & 0x00FF ) * 0.01 + 2.2
        data_dict["battery_6_voltage"] = (self.data.voltages[2] & 0x00FF ) * 0.01 + 2.2

        # MPPT
        #data_dict["mppt_temperature"] = (int(self.data.temperature) / 10) + 2731  # degC
        data_dict["mppt_charge_voltage"] = (self.data.voltages[3] & 0x0FFF) * 0.01
        data_dict["mppt_charge_current"] = (self.data.voltages[4] & 0x00FF) * 0.1

        data_dict["mppt_pv_voltage"] = (self.data.voltages[5] & 0x0FFF) * 0.01
        data_dict["mppt_pv_power"] = self.data.current_battery * -0.1

        return data_dict

    def to_influx_db(self):
        if self.data is None:
            return None
        else:
            dict_body = {}
            dict_body["measurement"] = self.data._type
            # local time in RFC 3339 format
            dict_body["time"] = datetime.now(timezone.utc).isoformat()
            dict_body["fields"] = self.data.to_dict()


            if self.data._type == 'PARAM_VALUE':
                dict_body["tags"] = {"param_id": dict_body["fields"]["param_id"]}

            if self.data._type == 'SYS_STATUS':
                dict_body["fields"]["load_voltage"] = dict_body["fields"]["voltage_battery"]/1000
                dict_body["fields"]["load_current"] = dict_body["fields"]["current_battery"]/100
                dict_body["fields"]["load_power"] = dict_body["fields"]["load_voltage"]*dict_body["fields"]["load_current"]

            if self.data._type == 'BATTERY_STATUS':
                if self.data.id == 1:
                    dict_body["fields"] = self._process_battery_status()
                    dict_body["measurement"] = "CREATEV_ELEC"
                    # print((self.data.current_battery))
                    # print(dict_body)
                else:
                    dict_body["fields"]["voltages"] = dict_body["fields"]["voltages"][0]
                    dict_body["measurement"] = "BATTERY_STATUS_" + str(self.data.id)

            return [dict_body]


while True:
    # Get data from mavlink
    mavlink_data = MavlinkData(master.recv_msg())
    influxdb_point = mavlink_data.to_influx_db()
    
    if influxdb_point is not None:
        point = Point.from_dict(influxdb_point[0])
        write_api.write(bucket, org, point)
        # print("suscess...")
    
master.close()
