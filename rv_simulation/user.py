""" User code lives here """
import time
from typing import Dict
import math
from typing import Callable, Optional
import numpy as np
import cv2
import glob
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion


class User:
    def __init__(self) -> None:
        self.pose = {
            "bravo_axis_a": 0.05,
            "bravo_axis_b": 0,
            "bravo_axis_c": math.pi * 0.5,
            "bravo_axis_d": math.pi * 0,
            "bravo_axis_e": math.pi * 0.75,
            "bravo_axis_f": math.pi * 0.9,
            "bravo_axis_g": math.pi
        }
        self.inc = 0.1
        self.last_time = time.time()

        return


    def run(self,
            image: list, 
            global_poses: Dict[str, np.ndarray],
            calcIK: Callable[[np.ndarray, Optional[np.ndarray]], Dict[str, float]],
            ) -> Dict[str, float]:
        """Run loop to control the Bravo manipulator.

        Parameters
        ----------
        image: list
            The latest camera image frame.

        global_poses: Dict[str, np.ndarray]
            A dictionary with the global camera and end-effector pose. The keys are
            'camera_end_joint' and 'end_effector_joint'. Each pose consitst of a (3x1)
            position (vec3) and (4x1) quaternion defining the orientation.
        
        calcIK: function, (pos: np.ndarray, orient: np.ndarray = None) -> Dict[str, float]
            Function to calculate inverse kinematics. Provide a desired end-effector
            position (vec3) and an orientation (quaternion) and it will return a pose
            dictionary of joint angles to approximate the pose.
        """
        
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #gb = cv2.GaussianBlur(image,(5,5),cv2.BORDER_DEFAULT)
        #edges = cv2.Canny(gb, 100, 200);


        #im2 = cv2.imread('drawingasjpeg.jpg')

        ## Detect April tags
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,
	        parameters=arucoParams)
        print(ids)

        ## Calibrate camera
        f = 201.383
        cameraMatrix = np.array([[f, 0, 0], [0, f, 0], [0, 0, 1]])
        distortMatrix = np.array([0.0, 0.0, 0.0, 0.0])
        markerLength = 0.06

        ## Get pose of April tags relative to camera pose
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, None)

        ## Apply operations to vector-quaternion representation of end effector pose
        if ids is not None and ids[0]==0:
            target_r =R.from_rotvec(rvecs[0][0])
            target_quat = R.as_quat(target_r)
            target_pos = tvecs[0][0]
            camera_pos = np.asarray(global_poses["camera_end_joint"][0])
            camera_quat = Quaternion(global_poses["camera_end_joint"][1])
            check = np.array([1, 0, 0])
            
            final_quat = camera_quat.rotate(Quaternion(target_quat))
            final_pos = camera_quat.rotate(target_pos)
            # final_pos[0]+= 0.07
            # final_pos[1]+= -0.03
            # final = np.array([final_pos[1],final_pos[0],final_pos[2]])
            self.pose = calcIK(final_pos,final_quat)
           # print (final_pos)
            # print(fina)


        #print("cam: ")
        #print(global_poses["camera_end_joint"])

        #print("end effector: ")
        #print(global_poses["end_effector_joint"])

        #print(quat)

        #print("r: ")
        #print(rvecs)
        #print("t: ")
        #print(tvecs)

        #x = cv2.rectangle(x, (413, 348), (528, 462), (255, 255, 255), -1)

        #x = cv2.rectangle(im2, (928, 348), (1042, 463), (200, 70, 30), 2)
        #z = cv2.bitwise_and(im2, im2, mask = x)

        #x = np.zeros([np.shape(image)[0], np.shape(image)[1]], dtype = 'uint8')
        #x = np.zeros([480, 630], dtype = 'uint8')
        #x = cv2.rectangle(x, (100, 100), (720, 720), (255, 255, 255), -1)
        #print([np.shape(image)[0], np.shape(image)[1]])
        #z = cv2.bitwise_and(image, image, mask = x)

        #if np.all(ids is not None):
        #    for i in range(0, len(ids)):
        #        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], markerLength, cameraMatrix, distortMatrix)
        #        #(rvec - tvec).any()
        #        cv2.aruco.drawDetectedMarkers(image, corners)  # Draw A square around the markers
        #        cv2.aruco.drawAxis(image, cameraMatrix, distortMatrix, rvec, tvec, 0.01)  # Draw Axis
        #        # Display the resulting frame
        #        cv2.imshow('View', image)
        #        cv2.waitKey(1)


        cv2.imshow("View", image)
        cv2.waitKey(1)
        
        # THIS IS AN EXAMPLE TO SHOW YOU HOW TO MOVE THE MANIPULATOR
        # bad while(len(ids) < 2):
        #if(ids is None):
        if self.pose["bravo_axis_g"] > math.pi:
            self.inc = -0.1
        
        if self.pose["bravo_axis_g"] < math.pi * 0.5:
            self.inc = 0.1
        
        self.pose["bravo_axis_g"] += self.inc

        # EXAMPLE USAGE OF INVERSE KINEMATICS SOLVER
        #   Inputs: vec3 position, quaternion orientation
        #print(global_poses["end_effector_joint"])
        #self.pose = global_poses[1][1] + tvecs
        #self.pose = calcIK(np.array([0.8, 0, 0.4]), np.array([1, 0, 0, 0]))

        return self.pose