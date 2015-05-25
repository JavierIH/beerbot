import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
objp *= 26.35

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Calibration set prefix
filePrefix = 'logitech/calibration_image'
images = glob.glob(filePrefix+'*.png')

for name in images:
    print "Found image: %s" % name

for i, fname in enumerate(images):
    print "Processing image %d" % i
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9, 6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print "\tPattern found!"

        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)


        # Draw and display the corners
        if corners2:
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        else:
            print 'Could not calculate sub-pixel corners, using normal corners'
            imgpoints.append(corners)
            cv2.drawChessboardCorners(img, (9,6), corners,ret)

        cv2.imshow('img',img)
        cv2.waitKey(200)
    else:
        print "\tCould not find pattern"


cv2.destroyAllWindows()


# Actual camera calibration:
print "Calibrating camera..."
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

print "Camera matrix: " + str(cameraMatrix)
print "Distortion coefficients: " + str(distCoeffs)

# Improving results:
img = cv2.imread(filePrefix + 'extra' + '.png')
h,  w = img.shape[:2]
newCameraMatrix, roi=cv2.getOptimalNewCameraMatrix(cameraMatrix,distCoeffs,(w,h),1,(w,h))

# Showing result:
print "Testing..."
src = cv2.imread(filePrefix+str(0)+'.png')
#src = cv2.imread('calibration_image2.png')
dst = cv2.undistort(src, cameraMatrix, distCoeffs, None, newCameraMatrix)
cv2.imshow('img',dst)
cv2.waitKey(-1)
cv2.destroyAllWindows()

# Calculating reprojection error:
print "Calculating reprojection error..."
tot_error = 0
for i in xrange(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    tot_error += error

print "\tTotal error: ", tot_error/len(objpoints)

# Save matrices:
np.savez(filePrefix+'.npz', cameraMatrix=cameraMatrix, distCoeffs=distCoeffs, newCameraMatrix=newCameraMatrix, roi=roi)