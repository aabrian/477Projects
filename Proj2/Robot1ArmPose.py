import robomaster
from robomaster import robot
import time
def sub_data_handler(sub_info):
    pos_x, pos_y = sub_info
    if pos_y>1000:
        pos_y = pos_y-2**32
    print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))
def pickup():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    
    # ep_arm.moveto(x=100, y=20).wait_for_completed()
    ep_arm.moveto(x=180, y=-70).wait_for_completed()

    ep_gripper.close(power=100)
    time.sleep(1)
    ep_gripper.pause()
    
    # ep_arm.moveto(x=170, y=-0).wait_for_completed()


    # ep_arm.move(x=-40, y=0).wait_for_completed()
def putdown():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_arm.moveto(x=180, y=-70).wait_for_completed()
    ep_gripper.open(power=100)
    time.sleep(1)
    ep_gripper.pause()
def openGripper():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_gripper = ep_robot.gripper
    ep_gripper.open(power=100)
    time.sleep(1)
    ep_gripper.pause()

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_arm.sub_position(freq=5, callback=sub_data_handler)
    pickup()
    # putdown()
    openGripper()
    # time.sleep(1)
    ep_arm.unsub_position()

    # time.sleep(10)
    # ep_gripper.pause()
    # ep_arm.moveto(x=180, y=-70).wait_for_completed()
    # ep_gripper.open(power=100)
    # time.sleep(1)
    # ep_gripper.pause()

    ep_robot.close()
