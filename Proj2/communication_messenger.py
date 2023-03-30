# ROBOT 1 is the messenger 

import zmq
import time
import robomaster
from robomaster import robot
from robomaster import config


context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.50.134:5555") # figure out IP adress stuff

def send_gripper_status(sub_info):
    time.sleep(3)
    global gripper_status
    gripper_status = sub_info
    global message
    message = gripper_status
    socket.send_string(message)
    print(message)

def Robot_destiniation(counter):
    time.sleep(3)
    if counter == 3:
        message = "arrived"
        print(message)
        socket.send_string(message)
        print(message)

    
#  Do 10 requests, waiting each time for a response

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta",sn = "3JKCH8800100WV")
    ep_gripper = ep_robot.gripper

    print("Connecting to recieving robot...")

    # finds lego grips lego and heads towards river once at river the counter is 3
    # so run Robot_destination which lets the other robot know that it is at the river
    Robot_destiniation(3)

    # now waits to recieve a reply from the reciever that it has gripped the lego
    # message = "False"
    Gripped = False
    while not Gripped:
        message = str(socket.recv())
        print("in while loop")
        if message == "True":
            Gripped = True
    # wait for a couple seconds then let go of the lego and send message to the recieving robot then this robot is done 
    time.sleep(3)
    ep_gripper.sub_status(freq = 1, callback = send_gripper_status)
    ep_gripper.open()
    time.sleep(3)
    ep_gripper.pause()



    


    # ep_gripper.unsub_status()



    # while True: #while loop might not be necessry depending on how movement works
    #     #robot goes to blocks and grips block
    #     ep_gripper.close()
    #     time.sleep(3)
    #     ep_gripper.pause()
    #     # get the current gripper status and sends it to reciever
    #     ep_gripper.sub_status(freq = 1, callback = send_gripper_status)
        

    # #     # if the status of the gripper is closed then send message to the recieving robot
    # #     if curr_status == robomaster.gripper.GripperStatus.Closed:
    # #         message = "Closed"
    # #         print("sending message to reciever robot")
    # #         socket.send_string(message)
    # #         # robot should now move towards the lake
         
    # #     #when the robot has arrived at the lake send a message to the reciever robot to let it know to grab the block 
    # #     message = "arrived"
    # #     socket.send_string(message)
    # #     #now the messanger robot should let go of the block and send a message alerting the recieving robot 
    # #     # ep_gripper.open(power=50)
    # #     # time.sleep(1)
    # #     # ep_gripper.pause()

    # #     if curr_status == robomaster.gripper.GripperStatus.OPEN:
    # #         message = "Open"
    # #         socket.send_string(message)
    # #         # messanger robot is done with all task
        
        
        
    ep_robot.close()