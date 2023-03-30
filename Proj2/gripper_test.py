import time
import robomaster
from robomaster import robot


def sub_data_handler(sub_info):
    # 完全闭合 closed, 完全张开opened, 处在中间位置normal.
    status = sub_info
    print("gripper status:{0}.".format(status))


if __name__ == '__main__':

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta",sn = "3JKCH8800100WV")

    ep_gripper = ep_robot.gripper
    ep_gripper.close(power=50)
    time.sleep(1)
    # 订阅机械爪状态
    ep_gripper.sub_status(freq=5, callback=sub_data_handler)
    ep_gripper.open()
    time.sleep(3)
    ep_gripper.close()
    time.sleep(3)
    ep_gripper.unsub_status()
    ep_robot.close()
