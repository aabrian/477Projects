#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    #  Wait for next request from client
    message = socket.recv()
    if message == "gripped":
        # code to have other robot start moving towards the lake to meet the recieving robot

    if message == "arrived":
        # grip the block 

    if message == "open":
        #can begin moving to the destination

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    socket.send(b"World")