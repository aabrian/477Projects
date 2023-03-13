<<<<<<< HEAD:dijkstra.py
# Project 1: Dijkstra
# Jared Scheffler - Section 0201

from matplotlib import pyplot as plt
import csv
import numpy
import time

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

def move_actual(c): # moves using inputed coordinate and direction
    y = c[1]
    x = c[0]
    plt.plot(x, y, marker="o", markersize=5, markerfacecolor="orange", markeredgecolor="orange")
    plt.pause(0.05)

if __name__ == '__main__':
 
    # opening and reading the CSV file
    file = csv.reader(open(rb'C:\Users\jsche\OneDrive - University of Maryland\Spring 2023\CMSC477\Project1\map.csv'), delimiter=',')
    x = list(file)
    maze = numpy.array(x).astype("int")

    numrows = len(maze)
    numcols = len(maze[0])

    plt.xlim(-5, numrows + 5)
    plt.ylim(-5, numcols + 5)

    # Loop creating maze background
    # note:* some additional math is needed because the indexing of a matrix and a graph is not exactly the same
    for i in range(numrows):
        for j in range(numcols):
            if maze[numcols-(i+1)][j] == 2: # starting point
                # plot red x at point
                plt.plot(j, i, marker="x", markersize=10, markerfacecolor="red", markeredgecolor="red")
                start_x = j
                start_y = i

            if maze[numrows-(i+1)][j] == 3: # goal
                # plot green circle at point
                plt.plot(j, i, marker="o", markersize=10, markerfacecolor="green", markeredgecolor="green")  
                goal_x = j
                goal_y = i            

            if maze[numrows-(i+1)][j] == 1: # wall
                # plot black at point
                plt.plot(j, i, marker="o", markersize=5, markerfacecolor="black", markeredgecolor="black")
    
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
    coord = (start_x,start_y)
    move(coord)
    k = costs[numrows-(start_y+1)][start_x]
    for i in range(k-1):
        new_c = check_surr_cost(k-i,coord)
        move(new_c)
        coord = new_c

    actual_path = [(start_x,start_y),(1,6),(1,5),(2,5),(3,5),(4,5),(5,5),(5,6),(5,7),(5,8),(6,8),(7,8),(8,8),(8,7),(8,6),(8,5),(8,4),(9,3),(10,4),(11,5),(11,6),(11,7)]
    for i in range(len(actual_path)):
        move_actual(actual_path[i])
    plt.show()

=======
# Project 1: Dijkstra
# Jared Scheffler - Section 0201

from matplotlib import pyplot as plt
import csv
import numpy
import time

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

if __name__ == '__main__':
 
    # opening and reading the CSV file
    file = csv.reader(open(rb'C:\Users\jsche\OneDrive - University of Maryland\Spring 2023\CMSC477\Project1\map.csv'), delimiter=',')
    x = list(file)
    maze = numpy.array(x).astype("int")

    numrows = len(maze)
    numcols = len(maze[0])

    plt.xlim(-5, numrows + 5)
    plt.ylim(-5, numcols + 5)

    # Loop creating maze background
    # note:* some additional math is needed because the indexing of a matrix and a graph is not exactly the same
    for i in range(numrows):
        for j in range(numcols):
            if maze[numcols-(i+1)][j] == 2: # starting point
                # plot red x at point
                plt.plot(j, i, marker="x", markersize=10, markerfacecolor="red", markeredgecolor="red")
                start_x = j
                start_y = i

            if maze[numrows-(i+1)][j] == 3: # goal
                # plot green circle at point
                plt.plot(j, i, marker="o", markersize=10, markerfacecolor="green", markeredgecolor="green")  
                goal_x = j
                goal_y = i            

            if maze[numrows-(i+1)][j] == 1: # wall
                # plot black at point
                plt.plot(j, i, marker="o", markersize=5, markerfacecolor="black", markeredgecolor="black")
    
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
    coord = (start_x,start_y)
    move(coord)
    k = costs[numrows-(start_y+1)][start_x]
    for i in range(k-1):
        new_c = check_surr_cost(k-i,coord)
        move(new_c)
        coord = new_c

    plt.show()

>>>>>>> 115b7a5b736eae86c5ad5831bd7386d771237d81:Proj1/dijkstra.py
