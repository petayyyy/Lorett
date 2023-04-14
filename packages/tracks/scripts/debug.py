from pyorbital.orbital import Orbital
from tle_storage import TleStrorage
from datetime import datetime, timedelta
from math import radians, tan, sin, cos
heigth = 0.77
# dt = datetime(2023, 4, 13, 12, 0, 0, 0)
dt = datetime.utcnow()

def sphericalToDecart(azimuth: float, elevation: float):
    if elevation == 90:
        return 0, 0     

    azimuth = radians(azimuth)
    elevation = radians(elevation)

    
    y = -(heigth / tan(elevation)) * cos(azimuth) 
    x = -(heigth / tan(elevation)) * sin(azimuth)
    
    return x, y

tle = TleStrorage().get_tle("NOAA 19")
orb = Orbital(satellite=tle.platform, line1=tle.line1, line2=tle.line2)

satPass = orb.get_next_passes(dt, 
                48, 68.993910, 61.001389, 60, 
                horizon=55)[0]
coordsX = []
coordsY = []
coordsZ = []

# Generating track steps


for i in range((satPass[1] - satPass[0]).seconds):

    dateTimeForCalc = satPass[0] + timedelta(seconds=i)
    observerLook = orb.get_observer_look(
        dateTimeForCalc, 68.993910, 61.001389, 60)
    print(observerLook)

    coords = sphericalToDecart(*observerLook)
    
    coordsX.append(coords[0])
    coordsY.append(coords[1])
    coordsZ.append(heigth)

    if abs(coords[0]) > 0.55 or abs(coords[1]) > 0.55:
        print("NIGGER")


track = list(zip(coordsX, coordsY, coordsZ))

for i in track:
    print(i)