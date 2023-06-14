#!/usr/bin/env python3.9
from lorettOrbital.scheduler import Scheduler, SchedulerConfig, sources
from datetime import datetime
from math import sin,cos,tan, pi, ceil
import rospy
from tracks.msg import TrackCoord, info
from time import time


STEP = 0.5

path: str = '.'


#class manager Schedule
class scheduleManage():
    nowTrackSphere = None
    nextTrackSphere = None

    nowSatPass = None
    nextSatPass = None

    track = None

    delay = None
    #init all
    def __init__(self, stationName, lat, lon, alt, horizon, minApogee, source, path, focus, delay) -> None:
        self.config = SchedulerConfig(stationName=stationName, 
                             lat=lat, 
                             lon=lon,
                             alt=alt,
                             horizon=horizon,
                             minApogee=minApogee,
                             source=source,
                             path=path)
        self.focus = focus
        self.delay = delay
        self.scheduler = Scheduler(config=self.config, logger=None)

        self.update()
        self.create_track(self.nowTrackSphere)
    
    #update satpass now and next
    def update(self):       
        now = datetime.utcnow()

        f = 12

        while(True):
            schedule = self.scheduler.getSchedule(timeStart=now, length=f, updateTLE=True)
            if(len(schedule) >= 2):
                break
            else:
                f += 1
        
        self.nowSatPass = schedule[0]
        self.nextSatPass = schedule[1]
        
        self.nowTrackSphere = self.scheduler.getSateliteTrack(satPass=schedule[0], timestep=STEP, save=True)
        self.nextTrackSphere = self.scheduler.getSateliteTrack(satPass=schedule[1], timestep=STEP, save=True)

        

        if(bool(rospy.get_param('~debug'))):
            devTime = self.nowTrackSphere[0].time.timestamp()
            for i in range(len(self.nowTrackSphere)):
                self.nowTrackSphere[i].time = datetime.fromtimestamp(datetime.utcnow().timestamp() + self.delay*2 + self.nowTrackSphere[i].time.timestamp() - devTime)
            devTime = self.nextTrackSphere[0].time.timestamp()
            for i in range(len(self.nextTrackSphere)):
                self.nextTrackSphere[i].time = datetime.fromtimestamp(self.nowTrackSphere[-1].time.timestamp() + self.delay*4 + self.nextTrackSphere[i].time.timestamp() - devTime)
        
        for i in self.nowTrackSphere:
            print(str(i.time.timestamp()) + ":" + str(i.azimuth) + ":" + str(i.elevation))
        
        
    
    #create info message from satpass
    def create_info(self, satPass):
        inf = info()
        inf.time_start = rospy.Time.from_sec(satPass.timeStart.timestamp())
        inf.time_stop = rospy.Time.from_sec(satPass.timeEnd.timestamp())
        inf.apogey = satPass.culmination
        inf.satname = satPass.satName

        return inf
    
    def create_track(self, track):

        tk = []
        for i in track:
            y=-(self.focus / tan(i.elevation/180 * pi)) * cos(i.azimuth/180 * pi)
            x=-(self.focus / tan(i.elevation/180 * pi)) * sin(i.azimuth/180 * pi)
            tk.append((x, y, i.time)) 
        self.track = tk.copy()

    def getStatus(self):
        now = datetime.utcnow().timestamp()
        if(now < self.getnowTrackSphere()[0].time.timestamp()):
            return "calling"
        elif(self.getnowTrackSphere()[-1].time.timestamp() > now > self.getnowTrackSphere()[0].time.timestamp()):
            return "receiving"
        elif(now > self.getnowTrackSphere()[-1].time.timestamp()):
            return "stop"

    def getPos(self):
        if(self.track[0][-1].timestamp() != self.getnowTrackSphere()[0].time.timestamp()):
            self.create_track(self.getnowTrackSphere())
        now = round(datetime.utcnow().timestamp() / STEP) * STEP

        i = max(0, round((now - self.track[0][-1].timestamp()) / STEP))

        return self.track[i]
    
    def getTimeSleep(self):
        now = ceil(datetime.utcnow().timestamp() / STEP) * STEP

        i = max(0, round((now - self.track[0][-1].timestamp()) / STEP))

        return self.track[i][-1].timestamp() - datetime.utcnow().timestamp()

    def getTimeSleepNext(self):
        return max(self.nextTrackSphere[0].time.timestamp() - datetime.utcnow().timestamp() - self.delay, 0)

    def getnowTrackSphere(self):
        if(datetime.utcnow().timestamp() >= self.nextTrackSphere[0].time.timestamp() - self.delay):
            self.update()
        return self.nowTrackSphere
            
        



if __name__ == "__main__":
    rospy.init_node("fly_node")
    coorsPub = rospy.Publisher("coords", TrackCoord, queue_size = 10)

    STEP = float(rospy.get_param('~step'))

    sm = scheduleManage(rospy.get_param('~stationName'), 
                   float(rospy.get_param('~lat')), 
                   float(rospy.get_param('~lon')),
                   float(rospy.get_param('~alt')),
                   int(rospy.get_param('~horizon')),
                   int(rospy.get_param('~minApogee')),
                   sources["weather"],
                   path,
                   float(rospy.get_param('~focus')),
                   int(rospy.get_param('~timeDelay')))
    
    inf = None

    rospy.sleep(sm.getTimeSleep() - sm.delay)

    while not rospy.is_shutdown(): 
        stat = sm.getStatus()
        print(stat)
        if(stat == "calling"):
            sm.getnowTrackSphere()
            inf = sm.create_info(sm.nowSatPass)

            tc = TrackCoord()
            
            tc.status = stat
            tc.info = inf

            tk = sm.getPos()

            tc.x = tk[0]
            tc.y = tk[1]
            tc.z = tk[2].timestamp()

            coorsPub.publish(tc)
            rospy.sleep(sm.getTimeSleep())
        elif(stat == "receiving"):
            tc = TrackCoord()
            
            tc.status = stat

            tc.info = inf

            tk = sm.getPos()

            tc.x = tk[0]
            tc.y = tk[1]
            tc.z = tk[2].timestamp()

            coorsPub.publish(tc)
            rospy.sleep(sm.getTimeSleep())
        elif(sm.getStatus() == "stop"):
            tc = TrackCoord()
            
            tc.status = 'stop'
            tc.info = sm.create_info(sm.nextSatPass)
            tc.x, tc.y, tc.z = (0,0,0)

            coorsPub.publish(tc)

            rospy.sleep(sm.getTimeSleepNext())
    rospy.spin()