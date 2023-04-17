import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config


#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.REQ)
#  you want to use the IP adress of thr computer connected to the robot you want to talk too
socket.connect("tcp://192.168.50.90:5555")
print("Connecting to ROBOT2…")