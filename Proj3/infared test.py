from robomaster import robot
from robomaster import camera
from matplotlib import pyplot
import time
import numpy as np
from scipy import interpolate
import csv
import pandas as pd
import math
distance_list = []
yaw_list = []

def sub_data_handler(sub_info):
    dist = sub_info
    # if pos_y>1000:
    #     pos_y = pos_y-2**32
    # print('dist = ' + str(dist(0)))#gives distance in mm
    # print('dist = ' + str(dist[0]))#gives distance in mm
    distance_list.append((dist[0],time.time()))

def sub_data_handler_attitude(sub_info):
    ang = sub_info
    # if pos_y>1000:
    #     pos_y = pos_y-2**32
    # print('angle = '+str(ang[0]))#gives distance in mm
    yaw_list.append((ang[0],time.time()))



ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")    
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
ep_sensor = ep_robot.sensor
ep_chassis = ep_robot.chassis


turn_angle = -45
turn_speed = 2.5


ep_chassis.move( x=0 , y=0 , z=turn_angle , xy_speed=0.5 , z_speed=turn_speed )
start = time.time()
ep_chassis.sub_attitude(freq = 10, callback = sub_data_handler_attitude)
ep_sensor.sub_distance(freq=10, callback = sub_data_handler)
while time.time()-start < 15:
    pass
ep_chassis.unsub_attitude()
ep_sensor.unsub_distance()

# strAngle = ""
# strDist = ""
# for result in yaw_list:
#     strAngle = strAngle + str(result.get_value()) + ","
# for result in distance_list:
#     strDist = strDist + str(result.get_value()) + ","    




angle = []
angle_time = []
# print(angle)
for i in yaw_list:
    angle.append(i[0])
    angle_time.append(i[1])

angle = np.unwrap(angle)


distance = []
distance_time = []
for i in distance_list:
    distance.append(i[0])
    distance_time.append(i[1])


# used to find when the angle will stop changing
diff = (np.diff(angle))


idx = [i for i,v in enumerate(diff) if abs(v)<.1]

if idx != [0]:

    angle = -angle[0:idx[1]]
    distance = distance[0:idx[1]]
    distance_time = distance_time[0:idx[1]]
    angle_time = angle_time[0:idx[1]]



f = interpolate.interp1d(angle_time,angle,fill_value='extrapolate')
new_angle = f(distance_time)

# print(new_angle)
# print(new_angle)
# print(new_angle.shape)

# if len(yaw_list)>len(distance_list):
#     yaw_list.pop()
# elif len(yaw_list)<len(distance_list):
#     distance_list.pop()
# # time.sleep(5)
# yaw_list = np.unwrap(yaw_list)
# yaw_list = np.array(yaw_list)
# distance_list= np.array(distance_list)
# print(yaw_list.shape)
# print(distance_list.shape)
# x = np.cos(yaw_list)*distance_list
# y = np.sin(yaw_list)*distance_list
fig = pyplot.figure()
ax = fig.add_subplot(211,projection='polar')
c = ax.scatter(np.pi*new_angle*(-1/180),distance)
# # pyplot.scatter(x,y)
ep_chassis.move( x=0 , y=0 , z=-1*turn_angle , xy_speed=0.5 , z_speed=turn_speed )

# plot the cartestian coords of what was captured
ax2 = fig.add_subplot(212)

# print(distance)
max_distance = 750
points = []
crit_angles = []
crit_distance = []

for i in range(len(distance)-1):
    if distance[i] < max_distance:
        if distance[i] < distance[i+1] and distance[i] < distance[i-1]:
            points.append(i)
        # elif distance[i] > distance[i+1] and distance[i] > distance[i-1]:
        #     points.append(i)

crit_xx = []
crit_yy = []

x_start = 13
y_start = 14
xx = np.cos(2*new_angle*np.pi/-180)*distance
yy = np.sin(2*new_angle*np.pi/-180)*distance
for i in range(len(points)):
    crit_xx.append(xx[points[i]])
    crit_yy.append(yy[points[i]])
    
for i in range(len(points)):
    print(distance[points[i]],new_angle[points[i]],crit_xx[i]/10,crit_yy[i]/10)
center = []
l_graph = 16
for i in range(len(crit_xx)):
    
    x = crit_xx[i]/10
    y = -crit_yy[i]/10
    print(x,y)
    if (math.ceil(abs(x)/l_graph)-abs(x/l_graph)) > (abs(x/l_graph)-math.floor(abs(x)/l_graph)):
        x_center = round((x_start*l_graph + x)/l_graph) - 0.5
    else:
        x_center = round((x_start*l_graph + x)/l_graph) + 0.5
    
    if (math.ceil(abs(y)/l_graph)-abs(y/l_graph)) > (abs(y/l_graph)-math.floor(abs(y)/l_graph)):
        y_center = round((y_start*l_graph + y)/l_graph) - 0.5
    else:
        y_center = round((y_start*l_graph + y)/l_graph) + 0.5
    
    center.append((x_center,y_center))
print(center)
d = ax2.scatter(yy/10,xx/10)
e = ax2.scatter(0,0,s=140)
ax2.set_aspect('equal')
ax2.set(xlim=(-200, 200), ylim=(0, 200))
# XandY_data = pd.DataFrame({'x':xx,'y':yy})
# XandY_data.to_csv('X and Y data.csv',index=False)
pyplot.show()
