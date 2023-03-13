import robomaster
from robomaster import robot
def sub_data_handler(sub_info):
    pos_x, pos_y = sub_info
    print("Robotic Arm: pos x:{0}, pos y:{1}".format(pos_x, pos_y))

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_arm = ep_robot.robotic_arm


    ep_arm.sub_position(freq=2, callback=sub_data_handler)
    # ep_arm.get_position()
    ep_arm.moveto(x=120, y=40).wait_for_completed()
    ep_arm.move(x=-40, y=0).wait_for_completed()
    ep_arm.unsub_position()
