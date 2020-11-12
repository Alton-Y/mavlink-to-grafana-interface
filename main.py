from pymavlink import mavutil
import time
from datetime import datetime, timezone
from influxdb import InfluxDBClient
import math


master = mavutil.mavlink_connection('udp:192.168.0.98:14561')
#master = mavutil.mavlink_connection('udp:192.168.0.103:14561')
master.wait_heartbeat()
time.sleep(2)

client = InfluxDBClient(host='192.168.0.99', port=8086)
#client.drop_database('CREATEV')
#client.create_database('CREATEV')
print(client.get_list_database())
client.switch_database('CREATEV')


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
        data_dict["voltage_1"] = self.data.voltages[0] / 1000
        data_dict["voltage_2"] = self.data.voltages[1] / 1000
        data_dict["voltage_3"] = self.data.voltages[2] / 1000
        data_dict["voltage_4"] = self.data.voltages[3] / 1000
        data_dict["voltage_5"] = self.data.voltages[4] / 1000
        data_dict["voltage_6"] = self.data.voltages[5] / 1000

        # MPPT
        data_dict["mppt_temperature"] = (int(self.data.temperature) / 10) + 2731  # degC
        data_dict["charge_current"] = math.floor(
            (65534 - int(self.data.current_consumed)) / 100) / 10  # Amp (VE_current_A)
        data_dict["charge_voltage"] = (65534 - int(self.data.current_consumed)) - data_dict[
            "charge_current"] * 1000  # Volt (VE_voltage_V)
        data_dict["charge_power"] = data_dict["charge_current"] * data_dict["charge_voltage"]

        # Solar Array
        data_dict["solar_voltage"] = -math.floor(int(self.data.current_battery) / 100 * 3)  # Volt          (VE_vpv_V)
        data_dict["solar_power"] = -(
                    int(self.data.current_battery) - (int(self.data.current_battery / 100) * 100)) * 4  # W (VE_ppv_W)
        if data_dict['solar_voltage'] > 0:
            data_dict["solar_current"] = data_dict["solar_power"] / data_dict["solar_voltage"]
        else:
            data_dict["solar_current"] = float(0)

        # Battery
        data_dict["battery_voltage"] = sum(self.data.voltages[0:6]) / 1000
        data_dict["battery_current"] = (int(self.data.current_battery) / 100) - data_dict["charge_current"]
        data_dict["battery_power"] = data_dict["battery_voltage"] * data_dict["battery_current"]
        # Load

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

            if self.data._type is 'PARAM_VALUE':
                dict_body["tags"] = {"param_id": dict_body["fields"]["param_id"]}

            if self.data._type is 'SYS_STATUS':
                dict_body["fields"]["load_voltage"] = dict_body["fields"]["voltage_battery"]/1000
                dict_body["fields"]["load_current"] = dict_body["fields"]["current_battery"]/100
                dict_body["fields"]["load_power"] = dict_body["fields"]["load_voltage"]*dict_body["fields"]["load_current"]

            if self.data._type is 'BATTERY_STATUS':
                if self.data.id == 2:
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
        client.write_points(influxdb_point)
        #print(influxdb_point)

master.close()
