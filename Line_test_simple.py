from pupil_apriltags import Detector
import cv2
import numpy as np
import time
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

def rotation_wa(theta):
    rot = np.array([[sin(theta),cos(theta),0], [0,0,-1], [-cos(theta),sin(theta),0]])
    return np.linalg.inv(rot)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    tag_size=0.2 # tag size in meters

    ep_chassis = ep_robot.chassis

    while True:
        try:
            img = ep_camera.read_cv2_image(strategy="newest", timeout=1)   
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray.astype(np.uint8)

            kk = 1.7
            K=np.array([[184.752*kk, 0, 320], [0, 184.752*kk, 180], [0, 0, 1]])

            results = at_detector.detect(gray, estimate_tag_pose=False)

            for res in results:
                pose = find_pose_from_tag(K, res)
                rot_ca, jaco = cv2.Rodrigues(pose[1], pose[1])
                # print('rotation = ')
                # print(rot_ca) # Rca - rotation matrix of 

                pts = res.corners.reshape((-1, 1, 2)).astype(np.int32)
                img = cv2.polylines(img, [pts], isClosed=True, color=(0, 0, 255), thickness=5)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                # print('x_pos = ')
                x_pos = pose[0][2]
                y_pos = pose[0][0]
                rot_wa = rotation_wa(np.pi)
                rot_ac = np.transpose(rot_ca)
                rot_wc = np.matmul(rot_wa, rot_ac)
                rot_bc = np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]])
                w2b = np.matmul(rot_bc,np.transpose(rot_wc))
                # R_bw = rot_cb.T @ 
                v_w = np.array([1,0,0])
                kb = 0.25
                v_b = kb*np.matmul(w2b,v_w)
                cv2.circle(img, tuple(res.center.astype(np.int32)), 5, (0, 0, 255), -1)
                pose[0][1]= 0
                Tag_loc = pose[0]
                Dtag_loc = [0, 0, -1]
                # print("Tag location ", Tag_loc)
                # print("Desired Tag location ", Dtag_loc)
                Tag_loc = Tag_loc/np.linalg.norm(Tag_loc)
                cross_product_AB = np.cross(Tag_loc, Dtag_loc)
                mag_cross = np.linalg.norm(cross_product_AB)
            
                dot_AB = np.dot(Tag_loc,Dtag_loc)
                # print("dot product", dot_AB)

                print(cross_product_AB)
                if pose[0][0] < 0:
                    theta = -(np.arctan2(mag_cross, dot_AB))*180/np.pi
                    #print("theta: ", theta)
                else:
                    theta = (np.arctan2(mag_cross, dot_AB))*180/np.pi
                    #print("theta: ", theta)
                kt = 11
                ep_chassis.drive_speed(x = v_b[1], y = v_b[0], z=kt*cross_product_AB[1], timeout=.5)
                # ep_chassis.drive_speed(x = -v_b[1], y = v_b[0], z=0, timeout=.5)
            cv2.imshow("img", img)
            cv2.waitKey(10)
           
        except KeyboardInterrupt:
            ep_camera.stop_video_stream()
            ep_robot.close()
            print ('Exiting')
            exit(1)