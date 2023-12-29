import cv2
import numpy as np
import apriltag
import pickle
import time

def camera_calibration():
    '''Function to access the intrinsic parameters of camera
    found using camera calibration'''

    file_path = 'CameraCalibration/cameraMatrix.pkl'
    with open(file_path, 'rb') as file:
        mtx = pickle.load(file)
    # print(mtx)

    fx = mtx[0][0] #focal length
    fy = mtx[1][1] #focal length
    cx = mtx[0][2] #focal centre
    cy = mtx[1][2] #focal centre

    with open('CameraCalibration/dist.pkl', 'rb') as file:
        dist_coeffs = pickle.load(file)

    return mtx, dist_coeffs

def findRT(r, mtx, dist_coeffs):
    '''For the given detected AprilTag result,
    it finds the AprilTag ID, rotation matrix and translation vector 
    from world frame to camera frame'''

    tag_id = r.tag_id
    # print("[INFO] Tag ID: {}".format(tag_id))

    det = apriltag.Detection(
        tag_family=r.tag_family,
        tag_id=tag_id,
        hamming=r.hamming,
        goodness=r.goodness,
        decision_margin=r.decision_margin,
        homography=r.homography,
        center=r.center,
        corners=r.corners
    )

    (ptA, ptB, ptC, ptD) = r.corners
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    tagFamily = r.tag_family.decode("utf-8")
    # print("[INFO] tag family: {}".format(tagFamily))

    corners = r.corners.astype(int)

    imagePoints = r.corners

    tag_size = 0.12
    tag_3d_points = np.array([
        [-tag_size/2, -tag_size/2, 0.0],
        [ tag_size/2, -tag_size/2, 0.0],
        [ tag_size/2,  tag_size/2, 0.0],
        [-tag_size/2,  tag_size/2, 0.0]
    ])

    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))

    midpt = ((ptB[0] + ptD[0])/2, (ptB[1] + ptD[1])/2)

    cv2.line(image, ptA, ptB, (0, 255, 0), 2)
    cv2.line(image, ptB, ptC, (0, 255, 0), 2)
    cv2.line(image, ptC, ptD, (0, 255, 0), 2)
    cv2.line(image, ptD, ptA, (0, 255, 0), 2)



    cv2.putText(image, str(tag_id), (int(midpt[0]), int(midpt[1])),
		cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
    imgPointsArr.append(imagePoints)
    objPointsArr.append(tag_3d_points)

    # mtx - the camera calibration's intrinsics
    good, prvecs, ptvecs = cv2.solvePnP(tag_3d_points, imagePoints, mtx, distCoeffs=dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    
    if good:
        rotation_matrix, _ = cv2.Rodrigues(prvecs)

    return tag_id, rotation_matrix, ptvecs, midpt

camera_coordinates = [[0,0,0], [0,0,1], [0,0,2]]
images = ['1.png', '2.png', '3.png']

# array storing all rotation and translation matrices found over time as [[{id, R, T, cam_location}]]
foundRT_overall = []

for idx in range(len(images)):
    foundRT_image = []
    
    path = images[idx]
    cam_coord = camera_coordinates[idx]
    print("Image:", path)
    mtx, dist_coeffs = camera_calibration()
    image = cv2.imread(path)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    height, width, _ = image.shape
    cv2.circle(image, (height//2, width//2), 5, (0, 0, 255), -1)

    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    print("[INFO] {} total AprilTags detected\n".format(len(results)))

    imgPointsArr = []
    objPointsArr = []

    for r in results:

        tag_id, R, T, midpt = findRT(r, mtx, dist_coeffs)
        print("TAG ID =", tag_id)
        
        camera_position = -np.dot(R.T, T)
        camera_orientation = R.T

        print("\nAssuming AprilTag location at (0,0,0), the data obtained is as follows:")
        print("Camera Position:", camera_position.T)
        print("Camera Orientation:")
        print(camera_orientation)
        print("Mid point at: ", midpt)
        print()

        foundRT_tag = {'id': tag_id, 'R': R, 'T': T, 'cam': cam_coord}
        foundRT_image.append(foundRT_tag)
    foundRT_overall.append(foundRT_image)

    cv2.imshow("Image", image)
    # time.sleep(10)
    # cv2.destroyAllWindows()
    key = cv2.waitKey(0)
    # key = input()
    if key == 27:
        cv2.destroyAllWindows()  # Close all OpenCV windows
    else:
        cv2.destroyWindow("Image") 

for i in foundRT_overall:
    print(i)