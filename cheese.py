from pupil_apriltags import Detector
import cv2
import csv
import numpy as np
import time
import csv
from robomaster import robot
from robomaster import camera
from math import(sin, cos, asin, pi, atan2, atan, acos)

time_step = 2
k_time = 15/time_step
t_run = 0.1
last_step = 0
final_range = 0.05
interval = 10
current_tag = 32
counter1 = 0
counter2 = 1

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
def check_surr_cost(n,j,i):
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
    
    return next_c[0],next_c[1]

def interp(t):
    # Take in a desired time, output an interpolated coordinated at that time
        # 1. Find the two coordinates above and below the desired time from path
        # 2. Using interpolation, find the new x and y coordinate
    for i in range(len(path)):
        if t - path[i][2] < time_step and t - path[i][2] > 0:
            x_below = path[i][0]
            y_below = path[i][1]
            t_below = path[i][2]
        if path[i][2] - t < time_step and path[i][2] - t > 0:
            x_above = path[i][0]
            y_above = path[i][1]
            t_above = path[i][2]

    x_interp = x_below + (t - t_below)*(x_above - x_below)/(t_above - t_below)
    y_interp = y_below + (t - t_below)*(y_above - y_below)/(t_above - t_below)
    return x_interp,y_interp

def derivative(t): #find instantaneous derivative in x and y to retrieve desired speed for feed forward
    step = 0.1
    vxd = .265*(interp(t+step)[0]-interp(t)[0])/step
    vyd = .265*(interp(t+step)[1]-interp(t)[1])/step
    return vxd,vyd

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))
def find_robot_pos(tag_coord,tag_orientation,robot_tag_pose):
    world_pos = [0,0]
    if tag_orientation == 'right':
        world_pos[0] = tag_coord[0]+robot_tag_pose[2]
        world_pos[1] = tag_coord[1]-robot_tag_pose[0]
    if tag_orientation == 'left':
        world_pos[0] = tag_coord[0]-robot_tag_pose[2]
        world_pos[1] = tag_coord[1]+robot_tag_pose[0]
    if tag_orientation == 'down':
        world_pos[0] = tag_coord[0]-robot_tag_pose[0]
        world_pos[1] = tag_coord[1]+robot_tag_pose[0]
    if tag_orientation == 'up':
        world_pos[0] = tag_coord[0]+robot_tag_pose[2]
        world_pos[1] = tag_coord[1]-robot_tag_pose[0]        

        
    return world_pos


def rotation_wa(direction):
    if direction == 'right':
        theta = 0
    elif direction == 'left':
        theta = np.pi
    elif direction == 'down':
        theta = np.pi/2
    elif direction == 'up':
        theta = 3*np.pi/2
    rot = np.array([[sin(theta),cos(theta),0], [0,0,-1], [-cos(theta),sin(theta),0]])
    return np.linalg.inv(rot)


if __name__ == '__main__':
    # Creating interpolation and desired path
    file = csv.reader(open(rb'map.csv'), delimiter=',')
    x = list(file)
    maze = np.array(x).astype("int")

    numrows = len(maze)
    numcols = len(maze[0])

    for i in range(numrows):
        for j in range(numcols):
            if maze[numcols-(i+1)][j] == 2: # starting point
                start_x = j
                start_y = i

            if maze[numrows-(i+1)][j] == 3: # goal
                # plot green circle at point 
                goal_x = j
                goal_y = i            


    # Dijkstra Algorithm

    costs = np.zeros([numrows,numcols], dtype = int)
    costs[numrows-(goal_y+1)][goal_x] = 1 # add a one to ending point in costs
    visited = [(goal_x,goal_y)]
    n = 1
    
    maze[numrows-(goal_y+1)][goal_x] = 0 # now that we know end coordinates, end must be set to 0 so maze knows it's open
    maze[numrows-(start_y+1)][start_x] = 0 # now that we know end coordinates, start must be set to 0 so maze knows it's open
    while True:
        
        if costs[numrows-(start_y+1)][start_x] != 0: # stop while loop 
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
    # x = start_x
    # y = start_y
    # t = 0
    # L = 0.265
    # path = [(L*(start_x-11),L*(start_y-7),t)]
    # k = costs[numrows-(start_y+1)][start_x]
    # for i in range(k-1):
    #     new_x,new_y = check_surr_cost(k-i,x,y)
    #     t = t+time_step
    #     path.append((L*(new_x-11),L*(new_y-7),t))
    #     x = new_x
    #     y = new_y
    
    t = 0
    L = .265
    path = [(start_x,start_y),(1,6),(1,5),(1,4),(2,4),(3,4),(4,4),(5,4),(5,5),(5,6),(5,7),(5,8),(6,8),(7,8),(7,7),(8,7),(8,6),(8,5),(8,4),(9,4),(10,4),(10,5),(10,6),(11,6),(goal_x,goal_y)]
    for i in range(len(path)):
        path[i] = (L*(path[i][0]-11),L*(path[i][1]-7),t)
        t = t+time_step

    # Finding current position
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis

    tag_size = 0.2 # tag size in meters
    l = .265
    
    tag_coords = {32 : [-8.5*l,0],34:[-8*l,-1.5*l],33:[-7.5*l,0],31:[-7.5*l,2*l],35:[-6*l,2.5*l],
    36:[-4*l,2.5*l],42:[-2.5*l,2*l],44:[-2.5*l,0],46:[-2*l,-1.5*l],45:[-1.5*l,0],39:[-4.5*l,-2*l],41:[-4.5*l,-4*l],43:[-1.5*l,2*l],37:[-5*l,-0.5*l],30:[-8.5*l,2*l],38:[-5.5*l,-2*l],40:[-5.5*l,-4*l]}
    tag_orientation  = {30:'left',32:'left',34:'down',33:'right',31:'right',35:'down',36:'down',42:'left',
                        44:'left',46:'down',45:'right',43:'right',37:'up',38:'left',40:'left',39:'right',41:'right',13:'right'}
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=2)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            kk = 1.7
            K=np.array([[184.752*kk, 0, 320], [0, 184.752*kk, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)
            
            if len(results) == 0:
                ep_chassis.move(x = 0, y = 0, z = 45, z_speed = 30).wait_for_completed()

            poses = []
            for res in results:
                # if t_run > last_step + time_step and res.tag_id != 40:
                #     new_tag = (res.tag_id)
                #     last_step = t_run
                # if res.tag_id != 40:
                #     new_tag = (res.tag_id)
                # else:
                #     new_tag = current_tag
                
                current_tag = res.tag_id
                print(current_tag)
                ## Checking if tag id match tags of interest and turning at these

                if current_tag == 34 and counter1 == 0: # rotate 90 degrees 90 degrees if tag 34 seen
                    time.sleep(1) # want the last velocity command to continue moving so gives robot space
                    t_run = t_run - 0.5 # don't want to mess up timing
                    ep_chassis.move(x = 0, y = 0, z = 45, z_speed = 30).wait_for_completed()
                    counter1 = 1

                if current_tag == 45 and counter2 == 0: # rotate 90 degrees 90 degrees if tag 45 seen
                    time.sleep(1) # want the last velocity command to continue moving so gives robot space
                    t_run = t_run - 0.5 # don't want to mess up timing
                    ep_chassis.move(x = 0, y = 0, z = 45, z_speed = 30).wait_for_completed()
                    counter2 = 0

                
                pose = find_pose_from_tag(K, res)
                rot_ca, jaco = cv2.Rodrigues(pose[1], pose[1])

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)

                # Current position [x,y]
                world_pose = find_robot_pos(tag_coords[current_tag],tag_orientation[current_tag],pose[0])
                poses.append(world_pose)
                
                if current_tag == 45:
                    if curr_y < .05 or curr_y > -.05: # if y coordinate of robot camera even with goal, exit program
                        ep_chassis.drive_speed(x = v_b[1], y = v_b[0], z=0, timeout=1)
                        exit(1)

                ############### NOT USING ##################
                # # Finding correct heading
                # pose[0][1]= 0
                # Tag_loc = pose[0]
                # if tag_orientation[current_tag] == 'right':
                #     Dtag_loc = [0, 0, 1]
                # elif tag_orientation[current_tag] == 'left':
                #     Dtag_loc = [0, 0, 1]
                # elif tag_orientation[current_tag] == 'down':
                #     Dtag_loc = [0, 0, 1]
                # elif tag_orientation[current_tag] == 'up':
                #     Dtag_loc = [0, 0, 1]
                

                # cross_product_AB = np.cross(Tag_loc, Dtag_loc)
                # mag_cross = np.linalg.norm(cross_product_AB)
            
                # dot_AB = np.dot(Tag_loc,Dtag_loc)


                # if pose[0][0] < 0:
                #     theta = -(np.arctan2(mag_cross, dot_AB))*180/np.pi
                # else:
                #     theta = (np.arctan2(mag_cross, dot_AB))*180/np.pi
                # kt = 1
                #############################################
            
            # Feedback Loop to get desired world velocity
            Kc = .1
            x_pos_des = interp(t_run)[0]
            y_pos_des = interp(t_run)[1]
            sum_x = 0
            sum_y = 0
            for i in range(len(poses)):
                sum_x = sum_x + poses[i][0]
                sum_y = sum_y + poses[i][1]
            curr_x = sum_x/len(poses)
            curr_y = sum_y/len(poses)
            print(curr_x,curr_y)

            # Feedfoward
            x_vel_des = derivative(t_run)[0]
            y_vel_des = derivative(t_run)[1]

            # Control Law
            output_x = Kc*(x_pos_des - curr_x) + x_vel_des
            output_y = Kc*(y_pos_des - curr_y) + y_vel_des
            v_w = np.array([output_x,output_y,0])

            # Converting to v_b
            rot_wa = rotation_wa(tag_orientation[current_tag])
            rot_ac = np.transpose(rot_ca)
            rot_wc = np.matmul(rot_wa, rot_ac)
            rot_bc = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]])
            w2b = np.matmul(rot_bc,np.transpose(rot_wc))
            kb = 1
            v_b = kb*np.matmul(w2b,v_w)

            ep_chassis.drive_speed(x = v_b[1], y = v_b[0], z=0, timeout=1)
            time.sleep(0.25)

            t_run = t_run + time_step/interval

            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)