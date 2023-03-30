#Robot 2 is the reciever

import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
    #  Wait for next request from client
    message = socket.recv()
    time.sleep(1)
    if message == "arrived":
        print(message)
        # code for robot to start moving towards the river and also code to grip the lego 

        # once the robot has gripped send  "True" message to the messanger so that messanger robot 
        #knows to let go 
        message = "True"
        socket.send(message)
       

    if message == "open":
        print(message)
        # code for robot to start moving towards the goal
        


    time.sleep(1)
