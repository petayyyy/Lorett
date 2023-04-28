#!/usr/bin/env python3

import rospy
from tracks.msg import TrackCoord, info
from time import sleep, time
from datetime import datetime
from fly_track import Track

class FlyNode:
    def __init__(self) -> None:
        rospy.init_node("fly_node")
        self.coorsPub = rospy.Publisher("coords", TrackCoord, queue_size = 10)
        self.track = Track(rospy.get_param("~lat"), rospy.get_param("~lon"), rospy.get_param("~alt"))
    
    def main(self):
        while not rospy.is_shutdown():
            satname, dateStart, dateStop, trackPath, apogey = self.track.generateTrack()
            
            rospy.loginfo('\n'*100)
            rospy.loginfo(f"Satellite: {satname}")
            rospy.loginfo(f"Start at: {dateStart}")
            rospy.loginfo(f"Sleep: {max(0, (dateStart - datetime.utcnow()).seconds)}")
            
            inf = info()
            inf.time_start = rospy.Time.from_sec(dateStart.timestamp())
            inf.time_stop = rospy.Time.from_sec(dateStop.timestamp())
            inf.apogey = apogey
            inf.satname = satname
            
            if(rospy.get_param('~debug') != True):
                rospy.sleep(max(0, (dateStart - datetime.utcnow()).seconds - 60))
            
            
            tc = TrackCoord()
            
            tc.status = 'calling'
            tc.info = inf
            tc.x = trackPath[0][0]
            tc.y = trackPath[0][1]
            tc.z = trackPath[0][2]
            
            self.coorsPub.publish(tc)
            if(rospy.get_param('~debug') != True):
                rospy.sleep(max(0, (dateStart - datetime.utcnow()).seconds))
            else:
                rospy.sleep(60)

            for coords in trackPath:
                t = time()
                tc = TrackCoord()

                tc.status = 'receiving'
                tc.info = inf
                tc.x, tc.y, tc.z = coords
                self.coorsPub.publish(tc)

                rospy.sleep(min(1, (1 - (time() - t))))
            
            tc = TrackCoord()

            tc.status = 'stop'
            tc.info = inf
            tc.x, tc.y, tc.z = (0,0,0)
            self.coorsPub.publish(tc)
    
if __name__ == "__main__":
    node = FlyNode()
    node.main()
    
