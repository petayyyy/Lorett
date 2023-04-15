 
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
import rospy

USAGE = '''
SDRReader.py - Script for recording signal by meteorolog satellite.
Example:
  SDRReader.py --s 'Noaa 19' --t 100
  SDRReader.py -a | --auto
  SDRReader.py -h | --help
  ***Don't forget about " or ' then you have space in naming. Also you can see example, use --h or -help then you start program.
Usage:
  SDRReader.py [--s='<name>'] [--t=<sec>] [--p=<name>] [--out_iq=<bool>] [--out_log=<bool>] [--out_con=<bool>]
  SDRReader.py -h | --help
  SDRReader.py -a | --auto
Options:
  -h, --help             Show correct format parameters.
  -a, --auto             Flag working in autonov secions.
  --s='<name>'             Name of satellite you recording [default: calibr].
  --t=<sec>              Time recording track your satellite [default: 120].
  --p=<name>             Name of tracks path [default: tracks].
  --out_iq=<bool>        Flag creating iq file with data recording [default: True].
  --out_log=<bool>       Flag creating log file with signal data [default: True].
  --out_con=<bool>       Flag printing console useless data [default: True].
'''
SDR_CONFIGS = {
    'calibr' : { 'freq': 137e6, 'rssi_freq': [137e6,137e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0 }, #default calibration
    'NOAA 18' : { 'freq': 1707.0e6, 'rssi_freq': [1707.3e6,1707.7e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0 },
    'NOAA 19' : { 'freq': 1698.0e6, 'rssi_freq': [1698.3e6,1698.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0 },
    'METEOR-M 2' : { 'freq': 1700.0e6, 'rssi_freq': [1700.3e6,1700.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0 },
    'METEOR-M2 2': {'freq': 1700.0e6, 'rssi_freq': [1700.3e6,1700.8e6 ], "sample_rate" : 6.0e6, 'bw': 4e6, 'gain':12.0},
    'METOP-B' : { 'freq': 1701.3e6, 'rssi_freq': [1700.7e6,1701.9e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':10.0 },
    'METOP-C' : { 'freq': 1701.3e6, 'rssi_freq': [1700.7e6,1701.9e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':10.0 },
    'FENGYUN 3B' : { 'freq': 1704.5e6, 'rssi_freq': [1704.0e6,1705.0e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0 },
    'FENGYUN 3C' : { 'freq': 1701.4e6, 'rssi_freq': [1701.0e6,1701.8e6 ], "sample_rate" : 6.0e6, 'bw': 6e6, 'gain':12.0 },
}

class OSMO_SDR:   ## Soapy based SDR class
    def __init__(self, SDR_CONFIGS, ros_topic_log, device_str="airspy"):
        self.ros_topic_log = ros_topic_log
        
        self.NO_FFT = False
        self.SDR_CONFIGS = SDR_CONFIGS
        self.device_str = device_str
        self.verbose = True
        self.conf = None
        self.th = None
        self.stream_break = False
        self.state='init'
        self.config_name='calibr'
        self.rssi = 0.0
        self.rssi_log = []
        self.noise_level = 0.0
        self.calibrated_at = datetime.utcnow() #last fix of self.noise_level
        #
        self.spectr_F = None
        self.spectr_PSD = None
        self.FFT_PERIOD = 0.5 #set to 0 to disable FFT calculation
        self.FFT_SIZE=512
        self.FFT_AVERAGE = 4 #defines number of FFT_SIZE samples passed to fft()
        self.fft_samples = numpy.array([0]*self.FFT_SIZE, numpy.complex64)
        #
        try:
            self.sdr = SoapySDR.Device(device_str)
        except Exception as e:
            print ("ERROR: OSMO_SDR failed to init device: {} ( {} )".format(device_str,e))
            self.state='NO DEVICE'
            raise Exception(e)

        if self.verbose :
            print  ("Init SDR device [{}]".format(device_str))
            print ("|-sample rates: "+str(self.sdr.listSampleRates(SOAPY_SDR_RX, 0)))
            print ("|-BW:"+str(self.sdr.getBandwidthRange(SOAPY_SDR_RX, 0)))
            print ("|-Gains:"+str(self.sdr.listGains(SOAPY_SDR_RX, 0)))
    def correct_satellite(self, satellite="calibr"):
        if "N" in satellite.upper() and "1" in satellite and "9" in satellite: self.config_name = "NOAA 19"
        elif "N" in satellite.upper() and "1" in satellite and "8" in satellite: self.config_name = "NOAA 18"
        elif "F" in satellite.upper() and "C" in satellite.upper(): self.config_name = "FENGYUN 3C"
        elif "F" in satellite.upper() and "B" in satellite.upper(): self.config_name = "FENGYUN 3B"
        elif "M" in satellite.upper() and "C" in satellite.upper(): self.config_name = "METOP-C"
        elif "M" in satellite.upper() and "B" in satellite.upper(): self.config_name = "METOP-B"
        elif "M" in satellite.upper() and "2" in satellite: self.config_name = "METEOR-M 2"
        elif "M" in satellite.upper() and "2" in satellite: self.config_name = "METEOR-M2 2"
        elif satellite in self.SDR_CONFIGS: self.config_name = satellite
        else:
            print ("Your name of satellite incorrect or don't work with L-diapozon, take one of this list:")
            print ("NOAA 19/18, FENGYUN 3B/3C, METEOR-M/M2 2")
            self.config_name = "calibr"
            return True
        return False
    def __del__(self):
        self.sdr = None
        if self.verbose : print ("SDR destroy")
        pass
    def fft(self,nperseg ,samples,sample_rate,center_freq):
        f, Pxx = signal.welch(samples, sample_rate, nperseg =nperseg , scaling='spectrum',return_onesided=False)
        f, Pxx = fftshift(f), fftshift(Pxx)
        f = f + center_freq
        return f,10*numpy.log10(Pxx)
    def stream_threading(self,data_file='',log_file='',spectr_file=''):
        buff = numpy.array([0]*1024*32, numpy.complex64)
        # start streaming
        self.state='streaming'
        #--prepare IQ data stream file
        data_fid = None
        if data_file != '':
            data_fid = open(data_file, "wb")
            if self.verbose :print  ("streaming to [{0}]".format(data_file))
        #prepare spectrum file
        spectr_fid = None
        spectr_write_heading =True
        if spectr_file != '':
            spectr_fid = open(spectr_file, "wt")
            if self.verbose : print  ("spectrum to [{}]".format(spectr_file))
        #--activate streaming
        rxStream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        self.sdr.activateStream(rxStream)
        #
        rssi_f0ind=None
        rssi_f1ind=None
        t0 = time.time()
        prev_shot_time = time.time()
        while not self.stream_break:
            sr = self.sdr.readStream(rxStream, [buff], len(buff))

            if data_fid is not None: buff.tofile(data_fid)

            if self.FFT_PERIOD > 0 :
                if time.time() - prev_shot_time > self.FFT_PERIOD: #fetch a FFT shot
                    if self.NO_FFT:
                        prev_shot_time = time.time()
                        self.rssi_log.append([datetime.utcnow(),0.0,0.0])
                        if self.verbose:
                            print ("{}\t-".format(datetime.utcnow()))
                    else:
                        #self.fft_samples = buff[0:self.FFT_SIZE]
                        self.spectr_F,self.spectr_PSD=self.fft(self.FFT_SIZE,buff[0: self.FFT_AVERAGE*self.FFT_SIZE],self.conf["sample_rate"],self.conf['freq'])
                        prev_shot_time = time.time()
                        #-- calc rssi by frequency subrange
                        if 'rssi_freq' in self.conf:
                            if rssi_f0ind is None: #find range indexes, do once
                                for i in range(0,len(self.spectr_F)):
                                    if rssi_f0ind is None:
                                        if self.spectr_F[i]>self.conf['rssi_freq'][0]:
                                            rssi_f0ind=i
                                            rssi_f1ind=i+1
                                    else:
                                        if self.spectr_F[i]<self.conf['rssi_freq'][1]: rssi_f1ind=i
                                if rssi_f0ind is None: #not yet found
                                    print ("SDR WARNING: not found RSSI range {}-{} in {}-{}".format(self.conf['rssi_freq'][0],self.conf['rssi_freq'][1],self.spectr_F[0],self.spectr_F[-1]))
                                else:
                                    print ("SDR RSSI range {}[{}]-{}[{}] in {}-{}".format(self.conf['rssi_freq'][0],rssi_f0ind,self.conf['rssi_freq'][1],rssi_f1ind,self.spectr_F[0],self.spectr_F[-1]))
                            if rssi_f0ind is not None:
                                self.rssi = numpy.mean(self.spectr_PSD[rssi_f0ind:rssi_f1ind])
                                self.rssi_log.append([datetime.utcnow(),self.rssi,self.rssi-self.noise_level])
                                if self.verbose:
                                    self.ros_topic_log(rospy.Time.now(), "streaming", self.config_name, self.rssi, self.noise_level)
                                    print ("{}\t{:.1f}\t{:.1f}".format(datetime.utcnow(),self.rssi,self.rssi-self.noise_level))
                        #-- save spectrum
                        if spectr_fid is not None: #dump spectr to file
                            if spectr_write_heading: #write file header
                                spectr_write_heading = False
                                numpy.set_printoptions(threshold=numpy.inf, suppress=True)
                                spectr_fid.write("Start: {}\nSDR Config:{}\nFFT Size: {}\nSample rate: {}\nFrequency: {}\n".format(datetime.utcnow(),self.config_name,self.FFT_SIZE,self.conf["sample_rate"],self.conf['freq']))
                                spectr_fid.write("SDR RSSI range {}[{}]-{}[{}] in {}-{}\n".format(self.conf['rssi_freq'][0],rssi_f0ind,self.conf['rssi_freq'][1],rssi_f1ind,self.spectr_F[0],self.spectr_F[-1]))
                                spectr_fid.write("Fbins: {}\n\n".format(numpy.array2string((self.spectr_F)/1e6,max_line_width=numpy.inf, precision=3)))
                            #print numpy.array2string(self.spectr_PSD[0:self.FFT_SIZE:self.FFT_SIZE/16].astype(int),  precision=0)
                            spectr_fid.write("{}\t{:.1f}\t{}\n".format(datetime.utcnow(),self.rssi,numpy.array2string(self.spectr_PSD, max_line_width=numpy.inf, precision=2))) #,separator ='\t'
                #
        #shutdown the stream and close files
        self.sdr.deactivateStream(rxStream) #stop streaming
        self.sdr.closeStream(rxStream)
        if data_fid is not None: data_fid.close()
        if spectr_fid is not None: spectr_fid.close()
        if log_file != '':
            try:
                with open(log_file, "wt") as rssi_f:
                    rssi_f.write("#Configuration: {}\n".format(self.config_name))
                    for r in self.rssi_log:
                        rssi_f.write("{}\t{:.1f}\t{:.1f}\n".format(r[0],r[1],r[2]))
            except Exception as ew:
                print("rssi log exception: {}".format(ew))
        self.state='idle'
        self.ros_topic_log(rospy.Time.now(), "end", self.config_name, 0, self.noise_level)
        print ("==========SDR streaming end===========")

    def load_config(self, conf_file=None, config_id=0):
        if self.correct_satellite(conf_file):
            print ("OSMO_SDR ERROR: invalid config [{}]".format(conf_file))
            return False
        if self.verbose : print ("|-Loading SDR config {}//{}".format(self.config_name,config_id))
        conf=self.SDR_CONFIGS[self.config_name]
        self.conf = conf
        self.sdr.setSampleRate(SOAPY_SDR_RX, 0, conf["sample_rate"])
        if self.verbose: print("Actual Rx Rate %f Msps"%(self.sdr.getSampleRate(SOAPY_SDR_RX, 0)/1e6))
        self.sdr.setFrequency(SOAPY_SDR_RX, 0, conf['freq'])
        self.sdr.setBandwidth(SOAPY_SDR_RX, 0, conf['bw'])
        if 'airspy' in self.device_str:
            self.sdr.setGain(SOAPY_SDR_RX, 0, 'LNA',conf['gain'])
            self.sdr.setGain(SOAPY_SDR_RX, 0, 'MIX',conf['gain'])
            self.sdr.setGain(SOAPY_SDR_RX, 0, 'VGA',conf['gain'])
        time.sleep(1)
        self.state='configured'
        if self.verbose :
            print ("|-Freq:" + str(self.sdr.getFrequency(SOAPY_SDR_RX, 0)))
        return True

    def start(self,data_file='',log_file='',spectr_file=''):

        if self.verbose : self.ros_topic_log(rospy.Time.now(),"Starting SDR...", self.config_name, 0, self.noise_level)
        if self.conf is None:
            self.ros_topic_log(rospy.Time.now(), "ERROR: SDR is not properly configured before start", self.config_name, 0, self.noise_level)
            return False
        try:
            self.stream_break = False
            self.th = threading.Thread(target=self.stream_threading, args=(data_file,log_file,spectr_file), daemon=True).start()
            
        except Exception as e:
            self.ros_topic_log(rospy.Time.now(), "SDR threading start exception: {}".format(e), self.config_name, 0, self.noise_level)
        return True

    def stop(self):
        self.stream_break = True
        time.sleep(0.2)
        if self.verbose : self.ros_topic_log(rospy.Time.now(), "////Stop SDR//// {}".format(self.state), self.config_name, 0, self.noise_level)
        pass

    def calibrate(self):
        buff = numpy.array([0]*1024*32, numpy.complex64)
        #--activate streaming
        rxStream = self.sdr.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
        self.sdr.activateStream(rxStream)
        #
        rssi_f0ind=None
        rssi_f1ind=None
        t0 = time.time()
        prev_shot_time = time.time()
        avg_sum=0.0
        avg_count=0
        while time.time()-t0 < 1.0:
            sr = self.sdr.readStream(rxStream, [buff], len(buff))
            if self.FFT_PERIOD > 0 :
                    self.spectr_F,self.spectr_PSD=self.fft(self.FFT_SIZE,buff[0: self.FFT_AVERAGE*self.FFT_SIZE],self.conf["sample_rate"],self.conf['freq'])
                    #-- calc rssi by frequency subrange
                    if 'rssi_freq' in self.conf:
                        if rssi_f0ind is None: #find range indexes, do once
                            for i in range(0,len(self.spectr_F)):
                                if rssi_f0ind is None:
                                    if self.spectr_F[i]>self.conf['rssi_freq'][0]:
                                        rssi_f0ind=i
                                        rssi_f1ind=i+1
                                else:
                                    if self.spectr_F[i]<self.conf['rssi_freq'][1]: rssi_f1ind=i
                            if rssi_f0ind is None: #not yet found
                                print ("SDR WARNING: not found RSSI range {}-{} in {}-{}".format(self.conf['rssi_freq'][0],self.conf['rssi_freq'][1],self.spectr_F[0],self.spectr_F[-1]))
                            else:
                                print ("SDR RSSI range {}[{}]-{}[{}] in {}-{}".format(self.conf['rssi_freq'][0],rssi_f0ind,self.conf['rssi_freq'][1],rssi_f1ind,self.spectr_F[0],self.spectr_F[-1]))
                        if rssi_f0ind is not None:
                            self.rssi = numpy.mean(self.spectr_PSD[rssi_f0ind:rssi_f1ind])
                            avg_sum += self.rssi
                            avg_count += 1
                            #if self.verbose: print "{}\t{:.1f}".format(datetime.utcnow(),self.rssi)
                #
        #shutdown the stream and close files
        self.sdr.deactivateStream(rxStream) #stop streaming
        self.sdr.closeStream(rxStream)
        self.state='idle'
        #-- save collected noise level
        self.noise_level = 0
        if avg_count > 0:
                self.noise_level = avg_sum/avg_count
        #else: self.noise_level =  0
        self.calibrated_at = datetime.utcnow()
        if self.verbose: print("Calibrated noise level: {}".format(self.noise_level))
        print ("==========end of calibration===========")
        return self.noise_level

    def log_telemetry(self, passid='',satellite='', config='', duration = None, stop_time=None, log_file=None):
        """ monitor and log telemetry """
        t_start = time.time()
        try:
            fh = open(log_file, "w")
            fh.write("#Pass ID: {}\n#Satellite: {}\n#Configuration: {}\n#Start time: {:%Y-%m-%d %H:%M:%S}\n\n".format(
                passid,satellite,config,datetime.datetime.utcnow()))
            fh.write("#Time\tLevel\tLevel2\tSNR\tBER\n")
        except Exception as e:
            print ()
            return False

        if stop_time is None and duration is None : duration = 300 #default
        logging_running = True
        while logging_running :
            #read
            #state = self.get_state(keep_alive=True)
            logln = "{}\t{:.1f}\t{:.1f}\t{:.1f}\t{}".format("{:%H:%M:%S.%f}".format(datetime.datetime.utcnow())[:-4],
                state['level'],state['level_out_dbm'],state['SNR'], state['BER'])
            print (logln)
            fh.write(logln + "\n")
            #write
            time.sleep(0.1)
            #
            if (( stop_time is not None and stop_time < datetime.datetime.utcnow())
                or ( duration is not None and (time.time() - t_start > duration ) ) ) :
                logging_running = False
                print ("stop {}".format(time.time() - t_start ))
        fh.write("\n#Closed at: {:%Y-%m-%d %H:%M:%S}".format(datetime.datetime.utcnow()))
        fh.close()

    def log_telemetry_threading(self, passid='',satellite='', config='', duration = None, stop_time=None,  log_file=None):
        """ run logging in a separate threading """
        #th = thread.start_new_thread(self.log_telemetry,(passid,satellite,config, duration,stop_time, log_file))
        th = threading.Thread(target=self.log_telemetry, args=(passid,satellite,config, duration,stop_time, log_file), daemon=True).start()

if __name__ == '__main__':
    global satellite, time_recording, path
    opts = docopt(USAGE)
    satellite = opts['--s']
    time_recording = int(opts['--t'])
    path = opts['--p']
    OUTPUT_IQ_FLAG = bool(opts['--out_iq'])
    OUTPUT_LOG_FLAG = bool(opts['--out_log'])
    OUTPUT_CON_FLAG = bool(opts['--out_con'])

    # Start work with airspy-sdr
    sdr = OSMO_SDR(SDR_CONFIGS, None)
    # Configurate/calibrate sdr by name of satellite
    if sdr.load_config(satellite):
        sdr.calibrate()
        # Start recording some signal by satellite``
        # Generate file name of path recording
        fileName = datetime.utcnow().strftime("%Y%m%d_%H%M%S_") + sdr.config_name 

        # Start recording signal
        sdr.start(os.path.join(path, fileName + ".iq"), os.path.join(path, fileName + ".log"))
        # Wait untill we see satellite
        time.sleep(time_recording)
        # Stop recording, end of all process
        sdr.stop()
