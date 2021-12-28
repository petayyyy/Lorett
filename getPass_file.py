from orbital import Station
from datetime import datetime
import glob
import os
import json

# Generate track
station = Station("Clover-Ilya",lon=37.498770, lat=55.930620, azimuthCorrection=89, alt=0.184)
station.findPasses(datetime.utcnow(), 30, saveTrack=True, printTrack= False)

list_of_files = glob.glob('tracks/*') # * means all if need specific format then *.csv
latest_file = max(list_of_files, key=os.path.getctime)
zz = 0.94

with open(latest_file, "r") as f:
    k, d = 0, []
    for i in f: 
        k+=1
        if k == 1: satellite_name = i[11:-1]
        if k == 5: x_apogee, y_apogee = map(float, i.split())
        if k >= 8:    d.append([*[int(j) for j in i.split()[0].split(":")], float(i.split()[-2]), float(i.split()[-1])])        

with open('/home/pi/config.py', 'w') as fw:
    fw.write("data = ")
    json.dump(d, fw)
    fw.writelines("\nx_apogee, y_apogee, zz = {0}, {1}, {2}".format(x_apogee, y_apogee, zz))
    fw.writelines("\nsateline_name = '{}'".format(satellite_name))
    fw.writelines("\ntime_delta = {}".format(  3600*(d[-1][0]-d[0][0]) + 60*(d[-1][1]-d[0][1]) + (d[-1][2] - d[0][2])))

