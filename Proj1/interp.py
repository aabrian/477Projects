import csv
import numpy

time_step = 1

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

if __name__ == '__main__':
        # opening and reading the CSV file
    file = csv.reader(open(rb'C:\Users\jsche\OneDrive - University of Maryland\Spring 2023\CMSC477\Project1\map.csv'), delimiter=',')
    x = list(file)
    maze = numpy.array(x).astype("int")

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

    costs = numpy.zeros([numrows,numcols], dtype = int)
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
    x = start_x
    y = start_y
    t = 0
    L = 0.265
    path = [(L*(start_x-11),L*(start_y-7),t)]
    k = costs[numrows-(start_y+1)][start_x]
    for i in range(k-1):
        new_x,new_y = check_surr_cost(k-i,x,y)
        t = t+time_step
        path.append((L*(new_x-11),L*(new_y-7),t))
        x = new_x
        y = new_y
    print(path)
    print(interp(.1))