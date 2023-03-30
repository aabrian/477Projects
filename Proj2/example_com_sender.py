import zmq
import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.50.134:5555") # figure out IP adress stuff
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta",sn = "3JKCH8800100WV")

context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.50.134:5555")

#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    socket.send(b"Hello")

    #  Get the reply.
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))