from pupil_apriltags import Detector
import cv2
import csv
import numpy as np
import time
import csv
from robomaster import robot
from robomaster import camera
from math import(sin, cos, asin, pi, atan2, atan, acos)

at_detector = Detector(
    families="tag36h11",
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)

def find_pose_from_tag(K, detection):
    m_half_size = tag_size / 2

    marker_center = np.array((0, 0, 0))
    marker_points = []
    marker_points.append(marker_center + (-m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, m_half_size, 0))
    marker_points.append(marker_center + ( m_half_size, -m_half_size, 0))
    marker_points.append(marker_center + (-m_half_size, -m_half_size, 0))
    _marker_points = np.array(marker_points)

    object_points = _marker_points
    image_points = detection.corners

    pnp_ret = cv2.solvePnP(object_points, image_points, K, distCoeffs=None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if pnp_ret[0] == False:
        raise Exception('Error solving PnP')

    r = pnp_ret[1]
    p = pnp_ret[2]

    return p.reshape((3,)), r.reshape((3,))

if __name__ == '__main__':
    tag_size = 0.2
    # Finding current position
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    counter = 0
    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=2)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            kk = 1.7
            K=np.array([[184.752*kk, 0, 320], [0, 184.752*kk, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)
            

            for res in results:
                pose = find_pose_from_tag(K, res)
                find_pose = pose[0]
                # rot, jaco = cv2.Rodrigues(pose[1], pose[1])
                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                if counter == 0 and res.tag_id == 32:
                    counter = counter + 1
                    ep_chassis.drive_speed(x = 0, y = 0.15, z=0, timeout=10)
                if counter == 1 and res.tag_id == 38:
                    if find_pose[0] < -0.22:
                        # Go forward
                        counter = counter + 1
                        ep_chassis.drive_speed(x = 0.15, y = 0, z=0, timeout=10)
                if counter == 2 and res.tag_id == 38:
                    if find_pose[2] < 0.5:
                        # Go Left
                        counter = counter + 1
                        ep_chassis.drive_speed(x = 0, y = -0.15, z=0, timeout=10)
                if counter == 3 and res.tag_id == 44:
                    if find_pose[0] > 0.22:
                        # Go forward
                        counter = counter + 1
                        ep_chassis.drive_speed(x = 0.15, y = 0, z=0, timeout=10)
                if counter == 4 and res.tag_id == 44:
                    if find_pose[2] < 0.5:
                        counter = counter + 1
                        ep_chassis.drive_speed(x = 0, y = 0.15, z=0, timeout=10)
                if res.tag_id == 46 and counter == 5:
                    if find_pose[0] < -0.3:
                        # Finding correct heading
                        pose[0][1]= 0
                        Tag_loc = pose[0]
                        Dtag_loc = [0, 0, 1]
                        cross_product_AB = np.cross(Tag_loc, Dtag_loc)
                        mag_cross = np.linalg.norm(cross_product_AB)
                        dot_AB = np.dot(Tag_loc,Dtag_loc)
                        if pose[0][0] < 0:
                            theta = -(np.arctan2(mag_cross, dot_AB))*180/np.pi
                        else:
                            theta = (np.arctan2(mag_cross, dot_AB))*180/np.pi
                        kt = .25
                        ep_chassis.drive_speed(x = 0, y = 0.1, z=kt*theta, timeout=10)
                    if find_pose[0] > 0:
                        counter = counter + 1
                        ep_chassis.drive_speed(x = .1, y = 0.15, z=0, timeout=10)
                if res.tag_id == 45 and counter == 6: 
                    if find_pose[0] < -0.25:
                        # Finding correct heading
                        pose[0][1]= 0
                        Tag_loc = pose[0]
                        Dtag_loc = [0, 0, 1]
                        cross_product_AB = np.cross(Tag_loc, Dtag_loc)
                        mag_cross = np.linalg.norm(cross_product_AB)
                        dot_AB = np.dot(Tag_loc,Dtag_loc)
                        if pose[0][0] < 0:
                            theta = -(np.arctan2(mag_cross, dot_AB))*180/np.pi
                        else:
                            theta = (np.arctan2(mag_cross, dot_AB))*180/np.pi
                        ep_chassis.drive_speed(x = 0, y = 0.1, z=kt*theta, timeout=10)
                    if find_pose[0] > 0:
                        counter = counter + 1
                        ep_chassis.drive_speed(x = .15, y = 0, z=0, timeout=10)
                if res.tag_id == 45 and counter == 7: 
                    if find_pose[2] < 0.5:
                        ep_chassis.drive_speed(x = 0, y = 0, z=0, timeout=10)
                        counter = counter + 1
            cv2.imshow("img", img)
            cv2.waitKey(10)

        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)