#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#

import zmq
import time
import robomaster
from robomaster import robot


context = zmq.Context()
print("Connecting to recieving robot...")
socket = context.socket(zmq.REQ)
socket.connect("tcp://10.104.16.30:5555") # figure out IP adress stuff

#  Do 10 requests, waiting each time for a response
if __name__ == 'main':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gripper = ep_robot.gripper
    prev_status = None

    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()

    while True: #while loop might not be necessry depending on how movement works
        #robot goes to blocks and grips block
        ep_gripper.closed(power=50)
        time.sleep(1)
        ep_gripper.pause()
        # get the current gripper status
        curr_status = ep_gripper.status

        # if the status of the gripper is closed then send message to the recieving robot
        if curr_status == robomaster.gripper.GripperStatus.Closed:
            message = "Closed"
            print("sending message to reciever robot")
            socket.send_string(message)
            # robot should now move towards the lake
         
        #when the robot has arrived at the lake send a message to the reciever robot to let it know to grab the block 
        message = "arrived"
        socket.send_string(message)
        #now the messanger robot should let go of the block and send a message alerting the recieving robot 
        ep_gripper.open(power=50)
        time.sleep(1)
        ep_gripper.pause()

        if curr_status == robomaster.gripper.GripperStatus.OPEN:
            message = "Open"
            socket.send_string(message)
            # messanger robot is done with all task
        
        
        



    ep_robot.close()