import numpy as np


# TODO equirectangular projection of latitude/longitude -> x/y
# https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
# https://stackoverflow.com/questions/3024404/transform-longitude-latitude-into-meters
# https://stackoverflow.com/questions/36254363/how-to-convert-latitude-and-longitude-of-nmea-format-data-to-decimal
def equirectangular_projection(latitude, longitude, lat_center=40.0104, lon_center=105.2443):
    #latitude_center = 40.0104
    #longitude_center = 105.2443
    poles_circumference = 40008000
    equator_circumference = 40075160
    y = (latitude - lat_center) * poles_circumference / 360
    x = (longitude - lon_center) * equator_circumference * np.cos(np.deg2rad(latitude)) / 360
    return x, y


'''
lat = 40.0103
lon = 105.2443
# every 0.0001 lat = 10 meters
# every 0.0001 lon = 8 meters
lat = 40.0107
lon = 105.2443

print(project(lat, lon))
'''


def convert(bytes):
    if 'G' in bytes:
        bytes = bytes.split(',')
        lat = (bytes[2])
        lon = (bytes[4])
        if not lon:
            print('No GPS data')

        else:
            lat = float(lat)
            lon = float(lon)
            lat_degree = int(float(lat) / 100)
            lng_degree = int(float(lon) / 100)

            lat_mm_mmmm = lat % 100
            lng_mm_mmmmm = lon % 100

            converted_latitude = lat_degree + (lat_mm_mmmm / 60)
            converted_longitude = lng_degree + (lng_mm_mmmmm / 60)
            print(converted_latitude)
            print(converted_longitude)
            print(bytes[9])
            return converted_latitude, converted_longitude, bytes[9]


gngga = '$GPGGA,224358.697,4000.661,N,10514.648,W,1,12,1.0,0.0,M,0.0,M,,*70'

lat, lon, alt = convert(gngga)

x, y = equirectangular_projection(lat, lon)

print(x, y)
