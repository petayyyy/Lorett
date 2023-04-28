#from main import station

import orb.orbital


name = "C4STest"
type = "c4s"
lat = 61.0014
lon = 68.9940
alt = 0.060
timeZone = 5

station = orb.orbital.Scheduler(name, lat, lon, alt,stationType=type, timeZone=timeZone, path="orb/")


lenght = 12

while(True):
    try:
        station.nextPass(length=lenght)
        break
    except:
        lenght+=1

sch = station.getSchedule(length=lenght, returnNameSatellite=True, updateTLE=False)

print(sch)

station.getSatellitePasses()
