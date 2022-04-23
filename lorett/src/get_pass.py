import SoapySDR
from SoapySDR import *
import sys
import os
import time
from datetime import datetime,timedelta
import numpy
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fftpack import fftshift
import threading
from docopt import docopt
from orbital import Station
import glob
import json

USAGE = '''

getPass_file - Script for generate list meteorolog satellite.

Example:
  getPass_file.py --lat 55.45 --lon 37.90 --alt 0.180 --t 10 
  getPass_file.py -a | --auto
  getPass_file.py -h | --help
  ***Don't forget about " or ' then you have space in naming. Also you can see example, use --h or -help then you start program.

Usage:
  getPass_file.py [--azim=<num>] [--alt=<num>] [--t=<sec>] [--lon=<num>] [--lat=<num>] [--name_st='<name>'] [--out_log=<bool>] [--out_con=<bool>]
  getPass_file.py -h | --help
  getPass_file.py -a | --auto

Options:
  -h, --help             Show correct format parameters.
  -a, --auto             Flag working in autonov secions.
  --t=<hour>             Parametr to find some satellite in time [default: 30].
  --lon=<num>            Name of satellite you recording [default: 37.498770].
  --lat=<num>            Time recording track your satellite [default: 55.930620].
  --azim=<num>           Name of tracks path [default: 0].
  --alt=<num>            Hight on Earth by sea [default: 0.184]
  --name_st='<name>'     Name station [default: 'C4S-007'].
  --path='<name>'        Name path creating python file with data satellite [default: tracks]
  --cr_py=<bool>         Flag creating python file with data satellite [default: True].
  --out_log=<bool>       Flag creating log file with signal data [default: True].
  --out_con=<bool>       Flag printing console useless data [default: True].

'''
def get_pass_server(req):
    try:
        if req.action == "calibrate":
            # Start work with airspy-sdr
            sdrr = OSMO_SDR(SDR_CONFIGS)
            # Configurate/calibrate sdr by name of satellite
            if req.satellite != '': 
                if sdrr.load_config(req.satellite): sdrr.calibrate()
            else: 
                if sdrr.load_config(satellite): sdrr.calibrate()
            return srv.sdr_recorder_rosResponse(process = "calibrate")
        elif req.action == "generate":
            try:
                if sdrr == None:
                    # Start work with airspy-sdr
                    sdrr = OSMO_SDR(SDR_CONFIGS)
                    # Configurate/calibrate sdr by name of satellite
                    if req.satellite != '': 
                        if sdrr.load_config(req.satellite): sdrr.calibrate()
                    else: 
                        if sdrr.load_config(satellite): sdrr.calibrate()
                    time.sleep(1)
            except: 
                # Start work with airspy-sdr
                sdrr = OSMO_SDR(SDR_CONFIGS)
                # Configurate/calibrate sdr by name of satellite
                if req.satellite != '': 
                    if sdrr.load_config(req.satellite): sdrr.calibrate()
                else: 
                    if sdrr.load_config(satellite): sdrr.calibrate()
                time.sleep(1)
            # Generate file name of path recording
            fileName = "{0}_{1:m%m_day%d_h%H_min%M_}".format(sdrr.config_name.replace(" ", "_"), datetime.utcnow())
            # Create path for all signals, if we don't have
            if req.path != '': 
                if not os.path.exists(req.path): os.makedirs(req.path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(req.path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(req.path, fileName, fileName)
            else: 
                if not os.path.exists(path): os.makedirs(path) 
                # Create path for now signal
                os.mkdir("{0}/{1}".format(path,fileName))
                # Generate file name of file recording
                fileName = "{0}/{1}/{2}".format(path, fileName, fileName)
            # Start recording signal
            sdrr.start("{0}.iq".format(fileName),"{0}.log".format(fileName),"")
            # Wait untill we see satellite
            if req.time_recording != 0: time.sleep(req.time_recording)
            else: time.sleep(time_recording)
            # Stop recording, end of all process
            sdrr.stop()
            return srv.sdr_recorder_rosResponse(process = "start")
        elif req.action == "start":
            # Generate data about your placement
            station = Station(req.name_station, lon = req.lon, lat = req.lat, azimuthCorrection = req.azimuth_correction, alt = req.alt)
            # Generate track
            station.findPasses(datetime.utcnow(), req.hour, saveTrack=True, printTrack= False)

            list_of_files = glob.glob(path+'/*') # * means all if need specific format then *.csv
            latest_file = max(list_of_files, key=os.path.getctime)
            # zz - is distance from drone to mirror
            zz = 0.94

            with open(latest_file, "r") as f:
                k, d = 0, []
                for i in f: 
                    k+=1
                    if k == 1: satellite_name = i[11:-1]
                    if k == 5: x_apogee, y_apogee = map(float, i.split())
                    if k >= 8:    d.append([*[int(j) for j in i.split()[0].split(":")], float(i.split()[-2]), float(i.split()[-1])])        
            if is_create_python_file:
                with open('/home/pi/Lorett/config.py', 'w') as fw:
                    fw.write("data = ")
                    json.dump(d, fw)
                    fw.writelines("\nx_apogee, y_apogee, zz = {0}, {1}, {2}".format(x_apogee, y_apogee, zz))
                    fw.writelines("\nsateline_name = '{}'".format(satellite_name))
                    fw.writelines("\ntime_delta = {}".format(  3600*(d[-1][0]-d[0][0]) + 60*(d[-1][1]-d[0][1]) + (d[-1][2] - d[0][2])))
        elif req.action == "kill" or req.action == "stop" or req.action == "exit":
            sdrr = None
            exit()
        else: return srv.sdr_recorder_rosResponse(process = "error")
    except:   return srv.sdr_recorder_rosResponse(process = "error")


if __name__ == '__main__':
    opts = docopt(USAGE)
    hour = int(opts['--t'])
    name_statiion = opts['--name_st']
    lon= float(opts['--lon'])
    lat =  float(opts['--lat'])
    azimuth_correction = int(opts['--azim'])
    alt = float(opts['--alt'])
    #is_create_python_file = bool(opts['--cr_py'])
    if not bool(opts['--auto']):
        # Generate data about your placement
        station = Station(name_statiion, lon = lon, lat = lat, azimuthCorrection = azimuth_correction, alt = alt)
        # Generate track
        station.findPasses(datetime.utcnow(), hour, saveTrack=True, printTrack= False)

        list_of_files = glob.glob(path+'/*') # * means all if need specific format then *.csv
        latest_file = max(list_of_files, key=os.path.getctime)
        # zz - is distance from drone to mirror
        zz = 0.94

        with open(latest_file, "r") as f:
            k, d = 0, []
            for i in f: 
                k+=1
                if k == 1: satellite_name = i[11:-1]
                if k == 5: x_apogee, y_apogee = map(float, i.split())
                if k >= 8:    d.append([*[int(j) for j in i.split()[0].split(":")], float(i.split()[-2]), float(i.split()[-1])])        
        if is_create_python_file:
            with open('/home/pi/Lorett/config.py', 'w') as fw:
                fw.write("data = ")
                json.dump(d, fw)
                fw.writelines("\nx_apogee, y_apogee, zz = {0}, {1}, {2}".format(x_apogee, y_apogee, zz))
                fw.writelines("\nsateline_name = '{}'".format(satellite_name))
                fw.writelines("\ntime_delta = {}".format(  3600*(d[-1][0]-d[0][0]) + 60*(d[-1][1]-d[0][1]) + (d[-1][2] - d[0][2])))
    else:
        try:
            rospy.init_node('get_pass_test')
            s = rospy.Service('sdr_recorder_ros', srv.get_pass_ros, get_pass_server) #////////////////////////////////////////////
            print("Ready generate list")
            rospy.spin()
        except: pass
