import csv
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera
import math
from scipy import interpolate
from matplotlib import pyplot as plt



counter = 0

center = []
angles = []
distance = []
visited = []

max_distance = 2250
points = []
crit_angles = []
crit_distance = []
crit_xx = []
crit_yy = []
distance_list = []
yaw_list = []
turn_angle = -45
turn_speed = 1.5

l_graph = 16 #side length of each box in csv in cm
edge_points = []

step = 2
process = 1
interval = 2
accepted_error = 0.1

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

def sub_position_handler(p, x_new):
    x_new[0] = p[0]
    x_new[1] = p[1]
    x_new[2] = p[2]
    # print("chassis position: x: {}".format(x_new))

def find_centers2(x,y,x_start,y_start):
    x = x/10
    y = y/10
    if (math.ceil(abs(x)/l_graph)-abs(x/l_graph)) > (abs(x/l_graph)-math.floor(abs(x)/l_graph)):
        x_center = round((x_start*l_graph + x)/l_graph) - 0.5
    else:
        x_center = round((x_start*l_graph + x)/l_graph) + 0.5
    
    if (math.ceil(abs(y)/l_graph)-abs(y/l_graph)) > (abs(y/l_graph)-math.floor(abs(y)/l_graph)):
        y_center = round((y_start*l_graph + y)/l_graph) - 0.5
    else:
        y_center = round((y_start*l_graph + y)/l_graph) + 0.5
    print(x,y,x_center,y_center)
    center.append((x_center,y_center))
    

def find_centers(angle,height,width,dist,x_start,y_start):
    ratio = height/width
    rad = dist
    x = rad*math.cos(angle)
    y = rad*math.sin(angle)
    if (math.ceil(x)-x) > (x-math.floor(x)):
        x_center = round((x_start*l_graph + x)/l_graph) + 0.5
    else:
        x_center = round((x_start*l_graph + x)/l_graph) - 0.5
    
    if (math.ceil(y)-y) > (y-math.floor(y)):
        y_center = round((y_start*l_graph + y)/l_graph) + 0.5
    else:
        y_center = round((y_start*l_graph + y)/l_graph) - 0.5
    if ratio <= 1: # single box
        center.append((x_center,y_center))
    elif ratio < 2 and ratio > 1: # double box
        if angle > 0 and angle < np.pi/2:
            center.append((x_center-1,y_center+1))
            center.append((x_center+1,y_center-1))
        elif angle > np.pi/2 and angle < np.pi:
            center.append((x_center+1,y_center+1))
            center.append((x_center-1,y_center-1))
        elif angle > np.pi and angle < np.pi*3/2:
            center.append((x_center-1,y_center+1))
            center.append((x_center+1,y_center-1))
        elif angle > np.pi*3/2 and angle < 2*np.pi:
            center.append((x_center+1,y_center+1))
            center.append((x_center-1,y_center-1))
    else: # clump, add a 2x2
        if angle > 0 and angle < np.pi/2:
            center.append((x_center-1,y_center+1))
            center.append((x_center+1,y_center-1))
            center.append((x_center-2,y_center+2))
            center.append((x_center+2,y_center-2))
        elif angle > np.pi/2 and angle < np.pi:
            center.append((x_center+1,y_center+1))
            center.append((x_center-1,y_center-1))
            center.append((x_center-2,y_center+2))
            center.append((x_center+2,y_center-2))
        elif angle > np.pi and angle < np.pi*3/2:
            center.append((x_center-1,y_center+1))
            center.append((x_center+1,y_center-1))
            center.append((x_center-2,y_center+2))
            center.append((x_center+2,y_center-2))
        elif angle > np.pi*3/2 and angle < 2*np.pi:
            center.append((x_center+1,y_center+1))
            center.append((x_center-1,y_center-1))
            center.append((x_center-2,y_center+2))
            center.append((x_center+2,y_center-2))

def add_walls(centers,maze):
    for i in range(len(centers)):
        x_center = centers[i][0]
        y_center = centers[i][1]
        maze[round(numrows-(y_center+0.5+1))][round(x_center+0.5)] = 1
        maze[round(numrows-(y_center-0.5+1))][round(x_center+0.5)] = 1
        maze[round(numrows-(y_center+0.5+1))][round(x_center-0.5)] = 1
        maze[round(numrows-(y_center-0.5+1))][round(x_center-0.5)] = 1
        maze[round(numrows-(y_center+1.5+1))][round(x_center+1.5)] = 1
        maze[round(numrows-(y_center-1.5+1))][round(x_center+1.5)] = 1
        maze[round(numrows-(y_center+1.5+1))][round(x_center-1.5)] = 1
        maze[round(numrows-(y_center-1.5+1))][round(x_center-1.5)] = 1
        maze[round(numrows-(y_center+0.5+1))][round(x_center+1.5)] = 1
        maze[round(numrows-(y_center-0.5+1))][round(x_center+1.5)] = 1
        maze[round(numrows-(y_center+0.5+1))][round(x_center-1.5)] = 1
        maze[round(numrows-(y_center-0.5+1))][round(x_center-1.5)] = 1
        maze[round(numrows-(y_center+1.5+1))][round(x_center+0.5)] = 1
        maze[round(numrows-(y_center-1.5+1))][round(x_center+0.5)] = 1
        maze[round(numrows-(y_center+1.5+1))][round(x_center-0.5)] = 1
        maze[round(numrows-(y_center-1.5+1))][round(x_center-0.5)] = 1
        for i in range(numrows):
            for j in range(numcols):
                if maze[numrows-(i+1)][j] == 1: # wall
                    # plot black at point
                    plt.plot(j, i, marker="o", markersize=5, markerfacecolor="black", markeredgecolor="black")

def check_surr(c): # open function to check if open in maze in 8 directions
    m = []
    i = c[1]
    j = c[0]
    if maze[numrows-(i+1)][j+1] == 0: # check right
        m.append((j+1,i)) # add right coord to list
    if maze[numrows-(i+2)][j+1] == 0: # check right and up
        m.append((j+1,i+1)) # add right and up coord to list
    if maze[numrows-(i+2)][j] == 0: # check up
        m.append((j,i+1)) # add above coord to list
    if maze[numrows-(i+2)][j-1] == 0: # check up and left
        m.append((j-1,i+1)) # add left and up to list
    if maze[numrows-(i+1)][j-1] == 0: # check left
        m.append((j-1,i)) # add left coord to list
    if maze[numrows-i][j-1] == 0: # check left and down
        m.append((j-1,i-1)) # add left and down to list
    if maze[numrows-i][j] == 0: # check down
        m.append((j,i-1)) # add below coord to list
    if maze[numrows-i][j+1] == 0: # check down and right
        m.append((j+1,i-1)) # add down and right to list
    return m

def check_visit(x,y): # checks is coordinate input is visited
    for i in range(len(visited)):
        if visited[i][0] == y and visited[i][1] == x:
            return True
    return False
def check_surr_cost(n,c):
    i = c[1]
    j = c[0]
    next_c = ()
    if costs[numrows-(i+1)][j+1] == n-1: # check right
        next_c = (j+1,i) # add right coord to list
    elif costs[numrows-(i+2)][j] == n-1: # check up
        next_c = (j,i+1) # add above coord to list
    elif costs[numrows-(i+1)][j-1] == n-1: # check left
        next_c = (j-1,i) # add left coord to list
    elif costs[numrows-i][j] == n-1: # check down
        next_c = (j,i-1) # add below coord to list
    elif costs[numrows-(i+2)][j-1] == n-1: # check up and left
        next_c = (j-1,i+1) # add left and up to list
    elif costs[numrows-(i+2)][j+1] == n-1: # check right and up
        next_c = (j+1,i+1) # add right and up coord to list
    elif costs[numrows-i][j-1] == n-1: # check left and down
        next_c = (j-1,i-1) # add left and down to list
    elif costs[numrows-i][j+1] == n-1: # check down and right
        next_c = (j+1,i-1) # add down and right to list
    else:
        print("Error, no coordinate found")
    
    return next_c

def move(c): # moves using inputed coordinate and direction
    y = c[1]
    x = c[0]
    plt.plot(x, y, marker="+", markersize=15, markerfacecolor="purple", markeredgecolor="purple")
    plt.pause(0.05)

def find_crit_points(start,goal):
    for i in range(numrows):
        for j in range(numcols):
            if maze[numrows-(i+1)][j] == start: # starting point
                # plot red x at point
                plt.plot(j, i, marker="x", markersize=10, markerfacecolor="red", markeredgecolor="red")
                x_start = j
                y_start = i
            if maze[numrows-(i+1)][j] == goal: # goal
                # plot green circle at point
                plt.plot(j, i, marker="o", markersize=10, markerfacecolor="green", markeredgecolor="green")  
                x_goal = j
                y_goal = i            
            if maze[numrows-(i+1)][j] == 1: # wall
                # plot black at point
                plt.plot(j, i, marker="o", markersize=5, markerfacecolor="black", markeredgecolor="black")
    return x_start,y_start,x_goal,y_goal

def dijkstra(x_start,y_start,x_goal,y_goal,maze):

    costs[numrows-(y_goal+1)][x_goal] = 1 # add a one to ending point in costs
    visited.append((x_goal,y_goal))
    n = 1
    
    maze[numrows-(y_goal+1)][x_goal] = 0 # now that we know end coordinates, end must be set to 0 so maze knows it's open
    maze[numrows-(y_start+1)][x_start] = 0 # now that we know end coordinates, start must be set to 0 so maze knows it's open
    while True:
        
        if costs[numrows-(y_start+1)][x_start] != 0: # stop while loop 
            break

        for k in range(len(visited)): # go through all visited cells in costs
            x = visited[k][0]
            y = visited[k][1]
            c = (x,y)
            if costs[numrows-(y+1)][x] == n: # check the cells for the desired number
                open = check_surr(c) # return open values in surounding 8 blocks in cardinal directions
                for i in range(len(open)):
                    c_check_x = open[i][1]
                    c_check_y = open[i][0]
                    c_check = open[i]
                    if not check_visit(c_check_x,c_check_y): # if it's not already assigned a cost
                        costs[numrows-(c_check[1]+1)][c_check[0]] = n+1 # add n at this coordinate to cost matrix
                        visited.append(c_check) # add this coordinate to visited matrix
        n = n + 1
    coord = (x_start,y_start)
    move(coord)
    k = costs[numrows-(y_start+1)][x_start]
    s = 0
    path = [(x_start,y_start,s)]
    for i in range(k-1):
        new_c = check_surr_cost(k-i,coord)
        s = s + step
        path.append((new_c[0],new_c[1],s))
        move(new_c)
        coord = new_c

    return path

def interp(t,path):
# Take in a desired time, output an interpolated coordinated at that time
    # 1. Find the two coordinates above and below the desired time from path
    # 2. Using interpolation, find the new x and y coordinate
    for i in range(len(path)):

        if t - path[i][2] < step and t - path[i][2] > 0:
            x_below = path[i][0]
            y_below = path[i][1]
            t_below = path[i][2]

        if path[i][2] - t < step and path[i][2] - t > 0:
            x_above = path[i][0]
            y_above = path[i][1]
            t_above = path[i][2]


    x_interp = x_below + (t - t_below)*(x_above - x_below)/(t_above - t_below)
    y_interp = y_below + (t - t_below)*(y_above - y_below)/(t_above - t_below)
    return x_interp,y_interp

if __name__ == '__main__':
    file = csv.reader(open(rb'477\Proj3\map_left1.csv'), delimiter=',')
    x = list(file)
    maze = np.array(x).astype("int")
    numrows = len(maze)
    numcols = len(maze[0])

    plt.ylim(-5, numrows + 5)
    plt.xlim(-5, numcols + 5)
    
    
    #### ROBOT SETUP CODE ####
    x_new = np.zeros((3,))

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_robot.chassis.sub_position(freq=25, callback=lambda p: sub_position_handler(p, x_new))
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
    frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))

    start = 3
    goal = 2
    crit_points = find_crit_points(start,goal)
  

    ############ FIND POINTS ###########

    ep_chassis.move( x=0 , y=0 , z=turn_angle , xy_speed=0.5 , z_speed=turn_speed)
    # ep_chassis.drive_speed(x=0,y=0,z=0,timeout = 10)
    start = time.time()
    ep_chassis.sub_attitude(freq = 10,callback = sub_data_handler_attitude)
    ep_sensor.sub_distance(freq = 10,callback = sub_data_handler)

    while time.time()-start < 15:
        pass
    ep_chassis.unsub_attitude()
    ep_sensor.unsub_distance()

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
        # print(type(angle))

        # used to find when the angle will stop changing
    diff = (np.diff(angle))
    # print(diff)
    idx = [i for i,v in enumerate(diff) if abs(v)<.1]

    if idx != [0]:

        angle = -angle[0:idx[1]]
        distance = distance[0:idx[1]]
        distance_time = distance_time[0:idx[1]]
        angle_time = angle_time[0:idx[1]]

    f = interpolate.interp1d(angle_time,angle,fill_value='extrapolate')
    new_angle = f(distance_time)

    ep_chassis.move( x=0 , y=0 , z=-1*turn_angle , xy_speed=0.5 , z_speed=turn_speed)

    # print(distance)
    points = []
    crit_angles = []
    crit_distance = []
    
    for i in range(1,len(distance)-1):
        print(distance[i],new_angle[i])
        if distance[i] < max_distance:
            if distance[i] < (distance[i+1]) and distance[i] < (distance[i-1]):
                points.append(i)
            # elif distance[i] > distance[i+1] and distance[i] > distance[i-1]:
            #     points.append(i)
    # for i in range(len(points)):
    #     crit_angles.append(new_angle[points[i]])
    #     crit_distance.append(distance[points[i]])
    xx = np.cos((2*new_angle)*np.pi/-180)*distance - 16
    yy = np.sin((2*new_angle)*np.pi/-180)*distance + 12.5
    for i in range(len(points)):
        crit_xx.append(xx[points[i]])
        crit_yy.append(yy[points[i]])


    for i in range(len(crit_xx)):
        # find_centers(crit_angles[i],1,1,crit_distance[i],find_crit_points[0](start,goal),find_crit_points[1](start,goal))
        find_centers2(crit_xx[i],-crit_yy[i],crit_points[0],crit_points[1])
    add_walls(center,maze)

    # Dijkstra Algorithm
    costs = np.zeros([numrows,numcols], dtype = int)
    des_path = dijkstra(crit_points[0],crit_points[1],crit_points[2],crit_points[3],maze)
    # plt.show()
    time.sleep(10)

    curr_x = crit_points[0]
    curr_y = crit_points[1]
    n = 0
    count = 0
    des_x_old = crit_points[0]
    des_y_old = crit_points[1]
    check_x = 0
    check_y = 0

    while abs(curr_x - crit_points[2]) > accepted_error or abs(curr_y - crit_points[3]) > accepted_error:
        ########### Wheel Odometry ############
        # Get current position
        
        curr_x = (crit_points[0]*l_graph - x_new[0]*100)/l_graph
        curr_y = (crit_points[1]*l_graph + x_new[1]*100)/l_graph
        boo = False
        print(n)
        for i in range(len(des_path)):
            if des_path[i][2] == n:
                des_x_new = des_path[i][0]
                des_y_new = des_path[i][1]
                boo = True
        if n == 0:
            des_x_new = des_path[0][0]
            des_y_new = des_path[0][1]
            boo = True
        if n == step*(len(des_path)-1):
            des_x_new = des_path[len(des_path)-1][0]
            des_y_new = des_path[len(des_path)-1][1]
            boo = True
        if not boo:    
            des_x_new = interp(n,des_path)[0]
            des_y_new = interp(n,des_path)[1]
        boo = False
        error_x = des_x_new - curr_x
        error_y = des_y_new - curr_y
        v_w = np.array([-error_x,-error_y,0])
        Rcw = np.array([[1,0,0],[0,-1,0],[0,0,1]])
        # time.sleep(0.25)
        v_b = np.matmul(Rcw,v_w)

        # if count > .25:
        #     print("curr_pos = " + str(curr_x) + ", " +str(curr_y))
        #     print('n = ' + str(n))
        #     print("des_pos = " + str(des_x_new) + ", " +str(des_y_new))
        #     print("error = " + str(error_x) + ", " +str(error_y))
        #     print("v_b = " + str(v_b))
        #     count = 0
        # else: count += .25
        print(curr_x,curr_y)
        print(crit_points[2],crit_points[3])
        print((curr_x - crit_points[2]),(curr_y - crit_points[3]))
        if abs(v_b[0]) < 0.25:
            Kpx = 0.5
        else: Kpx = 0.25
        if abs(v_b[0]) < 0.25:
            Kpy = 0.5
        else: Kpy = 0.25
        
        # ep_chassis.drive_speed(x = Kpx*v_b[0], y = Kpy*v_b[1], z = 0, timeout=1)
        time.sleep(0.05)
        if abs(error_x) < accepted_error and abs(error_y) < accepted_error:
            if n < step*(len(des_path)-1):
            # make a step to next desired position
            # ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                n = n + step/interval
            # print("Step reached, chaning n")

    ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=10)
    print("reached goal")