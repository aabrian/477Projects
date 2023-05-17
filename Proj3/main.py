import csv
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera
import math
from matplotlib import pyplot as plt



counter = 0

center = []
angles = []
distance = []
visited = []

l_graph = 16 #side length of each box in csv in cm
edge_points = []

step = 2
process = 1
interval = 2
accepted_error = 0.5

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
                # plt.plot(j, i, marker="x", markersize=10, markerfacecolor="red", markeredgecolor="red")
                x_start = j
                y_start = i
            if maze[numrows-(i+1)][j] == goal: # goal
                # plot green circle at point
                # plt.plot(j, i, marker="o", markersize=10, markerfacecolor="green", markeredgecolor="green")  
                x_goal = j
                y_goal = i            
            # if maze[numrows-(i+1)][j] == 1: # wall
                # plot black at point
                # plt.plot(j, i, marker="o", markersize=5, markerfacecolor="black", markeredgecolor="black")
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


def pickup():
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    

    ep_gripper.close(power=100)
    time.sleep(1)
    ep_gripper.pause()
    
    ep_arm.moveto(x=170, y=-0).wait_for_completed()

if __name__ == '__main__':
    file = csv.reader(open(rb'Proj3\map_left.csv'), delimiter=',')
    x = list(file)
    maze = np.array(x).astype("int")
    numrows = len(maze)
    numcols = len(maze[0])

    plt.ylim(-5, numrows + 5)
    plt.xlim(-5, numcols + 5)
    
    
    #### ROBOT SETUP CODE ####
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
    frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))

    #### LOOP STARTS HERE ####
    # Find Boxes and Add Walls
    if counter == 0:
        # Rotate 360 degrees, wheel odometry to get angle
        # Add angle and distance into arrays
        counter += 1
    if counter == 1:
        start = 3
        goal = 2
        crit_points = find_crit_points(start,goal)
        for i in range(len(angles)):
            find_centers(angles[i],1,1,distance[i],find_crit_points[0](start,goal),find_crit_points[1](start,goal)) 
        add_walls(center,maze)

        # Dijkstra Algorithm
        costs = np.zeros([numrows,numcols], dtype = int)
        des_path = dijkstra(find_crit_points(start,goal))
        counter += 1

    # plt.show()
    if counter == 2:
        # approach lego starting position
        counter += 1
    if counter == 3:
        # find first lego, go to it
        counter += 1
    if counter == 4:
        # pick up lego
        counter += 1
    if counter == 5:
        # return to lego starting position
        counter += 1
    if counter == 6:
        # return to starting position
        counter += 1
    if counter == 7:
        # wait for communication from other robot that it's in position
        counter += 1
    if counter == 8:
        pos = (crit_points[0],crit_points[1])
        
        while abs(pos[0] - crit_points[3]) > accepted_error and abs(pos[1] - crit_points[4]) > accepted_error:
            ########### Wheel Odometry ############
            # Get current position
            
            curr_x = pos[0]
            curr_y = pos[1]

            des_x = interp(n,des_path)[0]
            des_y = interp(n,des_path)[1]

            error_x = curr_x - des_x
            error_y = curr_y - des_y

            v_w = np.array([error_x,error_y,0])
            Rcw = np.array([[1,0,0],[0,-1,0],[0,0,1]])

            v_b = np.matmul(Rcw,v_w)

            Kp = 0.5
            ep_chassis.drive_speed(x = Kp*v_b[0], y = Kp*v_b[1], z = 0, timeout=1)
            if abs(error_x) < accepted_error and abs(error_y) < accepted_error:
                # make a step to next desired position
                n = n + step/interval
        counter += 1
    if counter == 9:
        # Goal is reached, communicate with other robot to come to river
        counter += 1
    if counter == 10:
        # center on river
        counter += 1
    if counter == 11:
        # approach river
        counter += 1
    if counter == 12:
        # wait for communication, release grip
        counter += 1
    if counter == 13:
        # return to position
        counter += 1
    if counter == 14:
        start = 2
        goal = 3
        crit_points = find_crit_points(start,goal)

        # Dijkstra Algorithm
        costs = np.zeros([numrows,numcols], dtype = int)
        des_path = dijkstra(find_crit_points(start,goal))

        pos = (crit_points[0],crit_points[1])
        
        while abs(pos[0] - crit_points[3]) > accepted_error and abs(pos[1] - crit_points[4]) > accepted_error:
            ########### Wheel Odometry ############
            # Get current position
            
            curr_x = pos[0]
            curr_y = pos[1]

            des_x = interp(n,des_path)[0]
            des_y = interp(n,des_path)[1]

            error_x = curr_x - des_x
            error_y = curr_y - des_y

            v_w = np.array([error_x,error_y,0])
            Rcw = np.array([[1,0,0],[0,-1,0],[0,0,1]])

            v_b = np.matmul(Rcw,v_w)

            Kp = 0.5
            ep_chassis.drive_speed(x = Kp*v_b[0], y = Kp*v_b[1], z = 0, timeout=1)
            if abs(error_x) < accepted_error and abs(error_y) < accepted_error:
                # make a step to next desired position
                n = n + step/interval
        
        counter = 1

    # cv2.imshow(frame,"frame")
    # cv2.waitKey(10)