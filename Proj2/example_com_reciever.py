import time
import zmq
import time
# import zmq
# import robomaster
# from robomaster import robot
# from robomaster import config

# ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    #  Wait for next request from client
    message = socket.recv()
    print(f"Recieved requst: {message}")

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    socket.send_string("World")