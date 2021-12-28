from orbital import Station
from datetime import datetime


# change coordinates
lon, lat, alt = 37.4988, 55.93056, 0.184
azimuthCorrection = 86
timeZone = 0

stationName = "Clover-Ilya"

station = Station(stationName, lon, lat, alt, timeZone, azimuthCorrection=azimuthCorrection)

station.findPasses(datetime.utcnow())
