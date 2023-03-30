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
    if message == "gripped":
        # code to have other robot start moving towards the lake to meet the recieving robot
        print(message)

    if message == "arrived":
        # grip the block 
        print(message)


    if message == "open":
        #can begin moving to the destination
        print(message)

    #  Do some 'work'
    time.sleep(1)
