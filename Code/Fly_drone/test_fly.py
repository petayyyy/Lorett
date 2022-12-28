import datetime
from config_fly import zz, fly_coordinat

time_fly = [11, 38, 31]
def navigate_wait(x = 0, y = 0, z = 1, fraime_id = 'body'):
    print('drone fly to {} {} {} by {}'.format(x, y, z, fraime_id))
def fly_point_file(file, zz):
    for f in file:
        print(f[1][0],  datetime.datetime.now().hour)
        if f[1][0] <= datetime.datetime.now().hour and f[1][1] <= datetime.datetime.now().minute and f[1][2] <= datetime.datetime.now().second:
            navigate_wait(x = file[k][0][0], y = file[k][0][1], z = zz, frame_id = "aruco_map")

fly_point_file(fly_coordinat, zz)
