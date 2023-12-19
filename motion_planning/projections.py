import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image


class Projector:
    def __init__(self, lat_center, lon_center):
        """
        https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
        https://stackoverflow.com/questions/3024404/transform-longitude-latitude-into-meters
        https://stackoverflow.com/questions/36254363/how-to-convert-latitude-and-longitude-of-nmea-format-data-to-decimal
        https://www.nmeagen.org/

        :param lat_center:
        :param lon_center:
        """
        self.poles_circumference = 40008000
        self.equator_circumference = 40075160
        self.latitude_center = lat_center
        self.longitude_center = lon_center
        self.TL_poi = None
        self.BR_poi = None

    def setup(self):
        self.TL_poi = PointOfInterest()
        self.TL_poi.px_x, self.TL_poi.px_y = 300, 68
        self.TL_poi.latitude, self.TL_poi.longitude = 40.01082206, 105.24452537
        self.TL_poi.x, self.TL_poi.y = self.equirectangular_projection(self.TL_poi.latitude, self.TL_poi.longitude)

        self.BR_poi = PointOfInterest()
        self.BR_poi.px_x, self.BR_poi.px_y = 710, 723
        self.BR_poi.latitude, self.BR_poi.longitude = 40.01027355, 105.24409488
        self.BR_poi.x, self.BR_poi.y = self.equirectangular_projection(self.BR_poi.latitude, self.BR_poi.longitude)

    def equirectangular_projection(self, latitude, longitude):
        y = (latitude - self.latitude_center) * self.poles_circumference / 360
        x = (longitude - self.longitude_center) * self.equator_circumference * np.cos(np.deg2rad(latitude)) / 360
        return x, y

    def project(self, latitude, longitude):
        poi = PointOfInterest()
        poi.latitude, poi.longitude = latitude, longitude
        poi.x, poi.y = self.equirectangular_projection(poi.latitude, poi.longitude)
        poi.px_x, poi.px_y = self.cartesian_to_pixel(poi.x, poi.y)
        return poi

    def cartesian_to_pixel(self, x, y):
        perx = (x - self.TL_poi.x) / (self.BR_poi.x - self.TL_poi.x)
        pery = (y - self.TL_poi.y) / (self.BR_poi.y - self.TL_poi.y)
        px = self.TL_poi.px_x + (self.BR_poi.px_x - self.TL_poi.px_x) * perx
        py = self.TL_poi.px_y + (self.BR_poi.px_y - self.TL_poi.px_y) * pery
        return px, py

    def get_pois(self):
        poi_a = PointOfInterest()
        poi_a.latitude = 40.01044911
        poi_a.longitude = 105.24441295

        poi_b = PointOfInterest()
        poi_b.latitude = 40.01075063
        poi_b.longitude = 105.24428344

        poi_c = PointOfInterest()
        poi_c.latitude = 40.01057135
        poi_c.longitude = 105.24421890

        poi_d = PointOfInterest()
        poi_d.latitude = 40.01031135
        poi_d.longitude = 105.24412890

        poi_home = PointOfInterest()
        poi_home.latitude = 40.01029409
        poi_home.longitude = 105.24434835

        for p in [poi_a, poi_b, poi_c, poi_d, poi_home]:
            p.x, p.y = self.equirectangular_projection(p.latitude, p.longitude)

        return poi_a, poi_b, poi_c, poi_d, poi_home


def equirectangular_projection(latitude, longitude, lat_center, lon_center):
    poles_circumference = 40008000
    equator_circumference = 40075160
    y = (latitude - lat_center) * poles_circumference / 360
    x = (longitude - lon_center) * equator_circumference * np.cos(np.deg2rad(latitude)) / 360
    return x, y


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
'''


class PointOfInterest:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.latitude = 0
        self.longitude = 0
        self.px_x = 0
        self.px_y = 0


def convert_poi(lat, lon):
    # https://www.nmeagen.org/
    lat_center = 40.01045433
    lon_center = 105.24432153

    TL_poi = PointOfInterest()
    TL_poi.px_x, TL_poi.px_y = 300, 68
    TL_poi.latitude, TL_poi.longitude = 40.01082206, 105.24452537
    TL_poi.x, TL_poi.y = equirectangular_projection(TL_poi.latitude, TL_poi.longitude,
                                                    lat_center=lat_center, lon_center=lon_center)

    BL_poi = PointOfInterest()
    BL_poi.px_x, BL_poi.px_y = 710, 723
    BL_poi.latitude, BL_poi.longitude = 40.01027355, 105.24409488
    BL_poi.x, BL_poi.y = equirectangular_projection(BL_poi.latitude, BL_poi.longitude,
                                                    lat_center=lat_center, lon_center=lon_center)

    poi = PointOfInterest()
    poi.latitude, poi.longitude = lat, lon
    poi.x, poi.y = equirectangular_projection(poi.latitude, poi.longitude,
                                              lat_center=lat_center, lon_center=lon_center)

    perx = (poi.x - TL_poi.x) / (BL_poi.x - TL_poi.x)
    pery = (poi.y - TL_poi.y) / (BL_poi.y - TL_poi.y)
    px = TL_poi.px_x + (BL_poi.px_x - TL_poi.px_x) * perx
    py = TL_poi.px_y + (BL_poi.px_y - TL_poi.px_y) * pery
    poi.px_x = px
    poi.px_y = py
    '''
    TL_pos = {'px': (68, 300), 'pos': (), 'll': (40.01082206, 105.24452537)}
    x, y = equirectangular_projection(TL_pos['ll'][0], TL_pos['ll'][1], lat_center=lat_center, lon_center=lon_center)
    TL_pos['pos'] = (y, x)
    
    BL_pos = {'px': (723, 710), 'pos': (), 'll': (40.01027355, 105.24409488)}
    x, y = equirectangular_projection(BL_pos['ll'][0], BL_pos['ll'][1], lat_center=lat_center, lon_center=lon_center)
    BL_pos['pos'] = (y, x)
    x, y = equirectangular_projection(latitude=lat, longitude=lon, lat_center=lat_center, lon_center=lon_center)
    pp = (y, x)

    P0 = TL_pos
    P1 = BL_pos

    perx = (pp[1] - P0['pos'][1]) / (P1['pos'][1] - P0['pos'][1])
    pery = (pp[0] - P0['pos'][0]) / (P1['pos'][0] - P0['pos'][0])
    print(perx, pery)
    px = P0['px'][1] + (P1['px'][1] - P0['px'][1]) * perx
    py = P0['px'][0] + (P1['px'][0] - P0['px'][0]) * pery
    '''
    return poi


if __name__ == '__main__':
    lat_center = 40.01045433
    lon_center = 105.24432153

    pp = Projector(lat_center, lon_center)
    pp.setup()
    pois = pp.get_pois()

    fig, ax = plt.subplots()
    for poi in pois:
        lat = poi.latitude
        lon = poi.longitude
        p = pp.project(lat, lon)

        img = Image.open('../base_interface/mission_area.png')
        img = np.asarray(img)
        plt.imshow(img)
        c = patches.Circle((p.px_x, p.px_y), radius=20, edgecolor='black')
        ax.add_patch(c)
        ax.axvline(x=p.px_x)
        ax.axhline(y=p.px_y)
    plt.show()
