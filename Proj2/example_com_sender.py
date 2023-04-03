import time
import zmq
import robomaster
from robomaster import robot
from robomaster import config


#  Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.50.134:5555")
print("Connecting to ROBOT2…")



#  Do 10 requests, waiting each time for a response
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="sta", sn = "3JKCH8800100WV")
ep_gripper = ep_robot.gripper



for request in range(10):
    print(f"Sending request {request} ...")
    socket.send_string("GRIPPED")
    ep_gripper.open(power=100)
    time.sleep(3)
    ep_gripper.pause()  


    #  Get the reply.
    message = socket.recv()
    print(f"Received reply {request} [ {message} ]")
    ep_gripper.open(power=100)
    time.sleep(3)
    ep_gripper.pause()  
