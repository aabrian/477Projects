#robot2


import time
import zmq
import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
ep_chassis = ep_robot.chassis
ep_arm = ep_robot.robotic_arm
ep_gripper = ep_robot.gripper
ep_arm = ep_robot.robotic_arm


context = zmq.Context()
socket = context.socket(zmq.PAIR)
print('binding')
socket.bind("tcp://*:5555")
print('done binding')

while True:
# listening for other robot when it starts rotating then start navigating to other robot
    print('waiting')
    message = socket.recv().decode()
    
    
    if message == "start":
        print("the message says: ", message)
        time.sleep(1)
        
        socket.send(b"started")
        
    

