import numpy as np
import cv2
import glob
import sys
import os

def main(args):

    if len(args) < 2:
        print("calibrate.py")
        print("\tThis program uses the imgaes inside a known file in jpg format to return an instrinsics matrix.")
        print("\tAt least 15 checkboard captures in different perspectives should be given")
        print("\tUse:")
        print("python3 calibrate.py directory")
        print("Output is given in intrinsics.yaml")
        return
    if not os.path.exists(args[1]):
        print("Directory " +args[1]+ " does not exist.")
        return

    directory = args[1].rstrip('/')
    # grid dimentions
    [n,m] = [6,9]

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    ## prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((n*m,3), np.float32)
    objp[:,:2] = np.mgrid[0:m,0:n].T.reshape(-1,2)

    ## Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob(directory+'/*.png') # al menos 15
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (m,n), None)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)
    cv2.destroyAllWindows()

    #   Calibration
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    #   Re-projection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )

    #   Save data
    f = open(directory+'/intrinsics.yaml', "w")
    f.write("camera_intrinsic_parameters: ")
    f.write(np.array2string(mtx.reshape(9), precision = 3, separator= ',', ))
    f.close()

    print("Intrinsics saved in "+directory+"/intrinsics.yaml")

if __name__ ==  "__main__":

    main(sys.argv)
