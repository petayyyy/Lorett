#!/usr/bin/env python3


from flask import Flask
import flask
import random
import json
import orb.orbital

import rospy
from tf2_msgs.msg import TFMessage
import subprocess



name = "C4STest"
type = "c4s"
lat = 55.6442
lon = 37.3325
alt = 0.159
timeZone = 3


RAD_REFLECTOR = 0.75

rospy.init_node('web_node')

Pos_x = 0
Pos_y = 0
Pos_z = 0



app = Flask(__name__)



@app.route('/')
def main():
    return flask.render_template("index.html")

@app.route('/reception')
def reception():
    return flask.render_template("reception.html")

@app.route('/satellite')
def satellite():
    station.update()
    # рассчитать ближайший пролет (сохраняет трек-файл рядом с собой)
    lenght = 12

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


@app.route('/get_pos')
def random_number():
    data = {
        "x":Pos_x,
        "y":Pos_y,
        "cellX": 1,
        "cellY": 0,
        "height": Pos_z
    }
    return json.dumps(data)

@app.route('/topic_request/<arg>')
def process_request(arg):
    # здесь можно обработать запрос и вернуть результат
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

def pos_callback(msg):
    global Pos_x, Pos_y, Pos_z
    if(msg.transforms[0].header.frame_id == "aruco_odom_map"):
        Pos_x = msg.transforms[0].transform.translation.x/RAD_REFLECTOR
        Pos_y = msg.transforms[0].transform.translation.y/RAD_REFLECTOR
        Pos_z = msg.transforms[0].transform.translation.z


if __name__ == '__main__':
    station = orb.orbital.Scheduler(name, lat, lon, alt,stationType=type, timeZone=timeZone, path="orb/")
    rospy.Subscriber('/tf', TFMessage, pos_callback)
    app.run(host="0.0.0.0", port=5000)
    rospy.spin()