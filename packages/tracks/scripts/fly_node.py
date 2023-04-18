#!/usr/bin/env python3

import rospy
from tracks.msg import TrackCoord
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
            satname, date, trackPath = self.track.generateTrack()
            
            rospy.loginfo('\n'*100)
            rospy.loginfo(f"Satellite: {satname}")
            rospy.loginfo(f"Start at: {date}")
            rospy.loginfo(f"Sleep: {max(0, (date - datetime.utcnow()).seconds)}")
            

            rospy.sleep(max(0, (date - datetime.utcnow()).seconds - 60)) # UNCOMENT FOR RELEASE
            
            tc = TrackCoord()
            
            tc.status = 'calling'
            tc.satname = satname
            tc.x = trackPath[0][0]
            tc.y = trackPath[0][1]
            tc.z = trackPath[0][2]
            
            self.coorsPub.publish(tc)

            rospy.sleep(max(0, (date - datetime.utcnow()).seconds))

            for coords in trackPath:
                t = time()
                tc = TrackCoord()

                tc.status = 'receiving'
                tc.satname = satname
                tc.x, tc.y, tc.z = coords
                self.coorsPub.publish(tc)
                #rospy.loginfo(f"{coords}")
                rospy.sleep(min(1, (1 - (time() - t))))
            
            tc = TrackCoord()

            tc.status = 'stop'
            tc.satname = satname
            tc.x, tc.y, tc.z = (0,0,0)
            self.coorsPub.publish(tc)
    
if __name__ == "__main__":
    node = FlyNode()
    node.main()
    