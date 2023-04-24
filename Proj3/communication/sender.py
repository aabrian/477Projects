#robot 1 
# bryans IP 192.168.50.90
#jareds IP 192.168.50.165

import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config


ep_robot = robot.Robot()
ep_chassis = ep_robot.chassis
ep_arm = ep_robot.robotic_arm
ep_gripper = ep_robot.gripper
ep_arm = ep_robot.robotic_arm

#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.REQ)
#  you want to use the IP adress of thr computer connected to the robot you want to talk too
socket.connect("tcp://:5555")
print("Connecting to ROBOT2…")



while True:
    # send message to robot when beginning heading towards the river letting robot 2 know to start moving 
    print("sending message to robot 2")
    socket.send(b"start")

    message = socket.recv().decode()
    print("message from robot 2: ", message)

    if message == "started":
        ep_gripper.close(100)
        time.sleep(1)