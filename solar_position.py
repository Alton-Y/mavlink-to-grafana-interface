from math import cos, sin, asin, pi, floor, atan2, tan, degrees, radians
from datetime import datetime, timezone, timedelta
from influxdb import InfluxDBClient

class SolarPosition:
    def __init__(self, GMTdatetime, Latitude, Longitude):

        theta = radians(Longitude) # convert Longitude input to radians
        psi = radians(Latitude) # convert Latitude input to radians

        # Grena 2012. Sec.3.1 - Starting point: time scale computation
        # t = datenum(y,m,d,h,0,0)-datenum(2060,0,0)-1 % identical to Grena 2012. p.1329 Eq.2
        t = GMTdatetime - datetime(2060, 1, 1)
        t = t.total_seconds()/24/60/60  # convert datetime to float days

        dtau = 96.4 + 0.00158 * t  # Grena 2012. p.1325 Eq.1
        te = t + 1.1574e-5 * dtau  # Grena 2012. p.1329 Eq.3

        #  Grena 2012. Sec.3.2 - Algorithm 1
        omega = 0.017202786  # [day^-1]

        alpha = -1.38880 + 1.72027920e-2*te \
            + 3.199e-2*sin(omega*te) - 2.65e-3*cos(omega*te) \
            + 4.050e-2*sin(2*omega*te) + 1.525e-2*cos(2*omega*te)  # Grena 2012. p.1329 Eq.4

        delta = 6.57e-3 + 7.347e-2*sin(omega*te) - 3.9919e-1*cos(omega*te) \
            + 7.3e-4*sin(2*omega*te) - 6.6e-3*cos(2*omega*te)  # Grena 2012. p.1329 Eq.5

        h = 1.75283 + 6.3003881*t + theta - alpha
        h = (h + pi)%(2.*pi) - pi  # H = mod(H + pi, 2.*pi) - pi

        e0 = asin(sin(psi)*sin(delta) + cos(psi)*cos(delta)*cos(h))  # Grena 2012. p.1332 Eq.21
        dpe = -4.26e-5*cos(e0)  # Grena 2012. p.1332 Eq.22
        ep = e0 + dpe  # elevation of the sun
        gamma = atan2(sin(h), cos(h)*sin(psi)-tan(delta)*cos(psi))  # Grena 2012. p.1332 Eq.23
        Z = pi/2 - ep  # Grena 2012. p.1332 Eq.25

        zenith = degrees(Z)
        elevation = degrees(ep)  # Elevation Angle in degrees
        azimuth = degrees(gamma)  # Azimuthal angle on the horizontal plane, 0 towards south, +ve towards west, and -ve towards east.
        heading = (azimuth+180) % 360  # Calculate heading bearing by adding 180 degrees to Azimuthal angle

        # [x,y,z] = sph2cart(az,elev,r)
        # sph2cart(-pi/2-Gamma, ep, 1)
        r = 1
        z = r * sin(ep)
        rcoselev = r * cos(ep)
        x = rcoselev * cos(-pi/2-gamma)
        y = rcoselev * sin(-pi/2-gamma)

        x = -x
        y = -y
        z = -z

        self.zenith = zenith
        self.elevation = elevation
        self.azimuth = azimuth
        self.heading = heading
        self.uvec = [x, y, z]


GMTdatetime = datetime(2020, 11, 13, 0, 0, 0)
# Latitude = 43.747156  # downsview
# Longitude = -79.475769 # downsview
Latitude = 43.838396  # York Soaring
Longitude = -80.43955 # York Soaring


solarPosition = SolarPosition(GMTdatetime, Latitude, Longitude)
print(solarPosition.uvec)

client = InfluxDBClient(host='192.168.0.99', port=8086)
#client.drop_database('CREATEV')
#client.create_database('CREATEV')
print(client.get_list_database())
client.switch_database('CREATEV')


while GMTdatetime < datetime(2020, 11, 15, 0, 0, 0):
    solarPosition = SolarPosition(GMTdatetime, Latitude, Longitude)

    dict_body = {}
    dict_body["measurement"] = 'SOLAR_MODEL'
    # local time in RFC 3339 format
    dict_body["time"] = GMTdatetime.isoformat()
    dict_body["fields"] = {"Latitude": Latitude,
                           "Longitude": Longitude,
                           "Zenith": solarPosition.zenith,
                           "Elevation": solarPosition.elevation,
                           "Azimuth": solarPosition.azimuth,
                           "Heading": solarPosition.heading}
    dict_body["tags"] = {"Location": 'YorkSoaring'}

    #print([dict_body])
    client.write_points([dict_body])
    GMTdatetime = GMTdatetime + timedelta(seconds=60)


