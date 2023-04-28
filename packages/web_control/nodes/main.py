#!/usr/bin/env python3


from flask import Flask
import flask
import json
import orb.orbital

import rospy
from geometry_msgs.msg import PoseStamped
import subprocess
from tracks.msg import TrackCoord



tracks = '/coords'


name = "C4STest"
type = "c4s"
lat = 61.0014
lon = 68.9940
alt = 0.060
timeZone = 5


RAD_REFLECTOR = orb.orbital.supportedStationTypes[type]["radius"]

rospy.init_node('web_node')

Pos_x = 0
Pos_y = 0
Pos_z = 0

cell_pos_x = 0
cell_pos_y = 0
cell_pos_z = orb.orbital.supportedStationTypes[type]["focus"]


satellite_name = ''


station = orb.orbital.Scheduler(name, lat, lon, alt,stationType=type, timeZone=timeZone, path="orb/")


app = Flask(__name__)


#!!!!!!!!!!!!!!!!!!!!!!1
def cmd_start():
    pass






@app.route('/')
def main():
    return flask.render_template("index.html")

@app.route('/reception')
def reception():
    return flask.render_template("reception.html")

@app.route('/satellite')
def satellite():
    station.update()
    lenght = 24

    while(True):
        try:
            station.nextPass(length=lenght)
            break
        except:
            lenght+=1

    return flask.render_template("satelite.html", station = station.getStation(), schedule = station.getSchedule(lenght, returnTd=True))

@app.route('/pos')
def pos():
    return "pos"

@app.route('/topic')
def topic():
    return flask.render_template("topic.html")

@app.route('/cmd')
def cmd():
    return flask.render_template("cmd.html")


@app.route('/get_pos')
def random_number():
    data = {
        "y":Pos_x,
        "x":Pos_y*-1,
        "cellY": cell_pos_x/0.55,
        "cellX": cell_pos_y/0.55,
        "sat": satellite_name,
        "height": Pos_z
    }
    #print(cell_pos_x + 0.55, cell_pos_y - 0.55)
    return json.dumps(data)

@app.route('/topic_request/<arg>')
def process_request(arg):
    if arg == 'all':
        arg = '<table>'
        for i in rospy.get_published_topics():
            arg += "<tr><td>"
            arg += i[0]
            arg += '</td><td>'
            arg += i[1]
            arg += '</td><tr>'
        arg += "</table>"
    else:
        arg = subprocess.getoutput(f"rostopic echo /{arg} -n 1").replace('\n', "<br>")
    return arg

@app.route('/start_cmd')
def web_start():
    try:
        cmd = flask.request.args.get("com")
        if(cmd == ''):
            cmd_start()
            return "OK"
        else:
            subprocess.run(cmd)
            return "OK"
    except:
        cmd_start()
    return "OK"

#!!!!!!!!!!!!!!!!!!!!!!!!
@app.route('/kill')
def kill_all():
    return "Kill"

def pos_callback(msg):
    global Pos_x, Pos_y, Pos_z
    Pos_x = (msg.pose.position.x - float(rospy.get_param('~pos_ref_x')))/RAD_REFLECTOR
    Pos_y = (msg.pose.position.y - float(rospy.get_param('~pos_ref_y')))/RAD_REFLECTOR
    Pos_z = msg.pose.position.z - float(rospy.get_param('~pos_ref_z'))
    #print(Pos_x, Pos_y, Pos_z)


def track_callback(msg):
    global cell_pos_y, cell_pos_x, satellite_name
    cell_pos_x = msg.x 
    #cell_pos_y = msg.y
    cell_pos_y = -1* msg.y
    satellite_name = msg.info.satname



if __name__ == '__main__':
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pos_callback)
    rospy.Subscriber(tracks, TrackCoord, track_callback)
    app.run(host="0.0.0.0", port=5000)
    rospy.spin()
