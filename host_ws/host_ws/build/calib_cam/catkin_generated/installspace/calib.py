import cv2
import numpy as np
import os
import glob
import rospy
from std_msgs.msg import Float32MultiArray 
import yaml
import math

def camera_calib():
    # Defining the dimensions of checkerboard
    CHECKERBOARD = (8,5)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
    # Creating vector to store vectors of 3D points for each checkerboard image
    objpoints = []
    # Creating vector to store vectors of 2D points for each checkerboard image
    imgpoints = [] 

    # Defining the world coordinates for 3D points
    objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    prev_img_shape = None
    
    # Extracting path of individual image stored in a given directory
    images = glob.glob('/home/jacky/Documents/Jacky/Calib/*.jpg')
    for frame in images:
        img = cv2.imread(frame)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        # If desired number of corners are found in the image then ret = true
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        """
        If desired number of corner are detected,
        we refine the pixel coordinates and display 
        them on the images of checker board
        """
        if ret == True:
            objpoints.append(objp)
            # refining pixel coordinates for given 2d points.
            corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
            
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        
        cv2.imshow('img',img)
        cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    
    h,w = img.shape[:2]
    
    """
    Performing camera calibration by 
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the 
    detected corners (imgpoints)
    """
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    img = cv2.imread('/home/jacky/Documents/Jacky/Calib/2023-05-04-152605.png')

    h,  w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    dst = dst[7:591, 90:710]
    cv2.imshow('123', dst)
    dst = cv2.Canny(dst, 210, 250, None, 3)
    cv2.imshow('Line', dst)
    # Copy edges to the images that will display the results in BGR
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
    
    lines = cv2.HoughLines(dst, 1, np.pi / 180, 250, None, 0, 0)
    
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(cdst, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
            print(str(i) + " " + str(pt1) + " " + str(pt2))
    cv2.imshow('Line detect', cdst)

    # cam_params.data = mtx
    mtx = mtx.flatten()
    mtx = np.append(mtx, dist)
    # mtx = np.append(mtx, tvecs)
    print(tvecs[0])
    print(mtx)
    cam_params.data = mtx
    # data = dict(Matrix = mtx)
    cam_data = {"data" : cam_params.data.tolist()}
    with open('/home/jacky/host_ws/src/image_convert/launch/camera_param.yaml', 'w') as f:
        yaml.dump(cam_data, f)
    cv2.waitKey(0)
    # cam_par = yaml.safe_load(cam_params)

    # print(cam_params)     

def main():
    camera_calib()

if __name__ == '__main__':
    rospy.init_node('calib_cam', anonymous = True)
    camera_pub = rospy.Publisher('/camera_params', Float32MultiArray, queue_size=10)
    cam_params = Float32MultiArray()
    main()
    