#robot2


import time
import zmq
import time
# import zmq
# import robomaster
# from robomaster import robot
# from robomaster import config

<<<<<<< HEAD
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
ep_gripper = ep_robot.gripper
=======
# ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
>>>>>>> f8bca357cfbebf9e33e6e74ac9a108435b8f8cfe

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")


ep_gripper.open(power=100)
time.sleep(1)
ep_gripper.pause()  
while True:
    #  Wait for next request from client
    
    message = socket.recv()
    print(f"Recieved requst: {message}")

    #  Do some 'work'
    time.sleep(1)

    #  Send reply back to client
    socket.send_string("World")