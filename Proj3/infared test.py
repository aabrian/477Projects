from robomaster import robot
from matplotlib import pyplot
import time
import numpy as np
from scipy import interpolate
import csv
import pandas as pd
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
ep_sensor = ep_robot.sensor
ep_chassis = ep_robot.chassis


turn_angle = -75
turn_speed = 2.5


ep_chassis.move( x=0 , y=0 , z=turn_angle , xy_speed=0.5 , z_speed=turn_speed )
start = time.time()
ep_chassis.sub_attitude(freq = 10,callback = sub_data_handler_attitude)
ep_sensor.sub_distance(freq=10,callback = sub_data_handler)
while time.time()-start <15:
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
print(angle)
distance = []
distance_time = []
for i in distance_list:
    distance.append(i[0])
    distance_time.append(i[1])
    # print(type(angle))

# used to find when the angle will stop changing
diff = (np.diff(angle))
print(diff)
# print('differnce')
# print(diff)
idx = [i for i,v in enumerate(diff) if abs(v)<.1]
print(idx)
if idx != [0]:

    angle = -angle[0:idx[1]]
    distance = distance[0:idx[1]]
    distance_time = distance_time[0:idx[1]]
    angle_time = angle_time[0:idx[1]]
print('new angles', angle)



f = interpolate.interp1d(angle_time,angle,fill_value='extrapolate')
new_angle = f(distance_time)


print(new_angle)
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

xx = np.sin(new_angle*np.pi/-180)*distance
yy = np.cos(new_angle*np.pi/-180)*distance
d = ax2.scatter(xx,yy)
e = ax2.scatter(0,0,s=140)
ax2.set_aspect('equal')
ax2.set(xlim=(-2000, 2000), ylim=(0, 2000))
# XandY_data = pd.DataFrame({'x':xx,'y':yy})
# XandY_data.to_csv('X and Y data.csv',index=False)
pyplot.show()
