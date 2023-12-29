import cv2
import numpy as np
import apriltag
import pickle

file_path = 'CameraCalibration/cameraMatrix.pkl'
with open(file_path, 'rb') as file:
    mtx = pickle.load(file)

with open('CameraCalibration/dist.pkl', 'rb') as file:
    dist_coeffs = pickle.load(file)

image = cv2.imread('id1.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(options)
results = detector.detect(gray)
print("[INFO] {} total AprilTags detected".format(len(results)))

imgPointsArr = []
objPointsArr = []

for r in results[:2]:  # Process the first two detected tags
    tag_id = r.tag_id
    print("[INFO] Tag ID: {}".format(tag_id))

    (ptA, ptB, ptC, ptD) = r.corners
    tag_size = 0.12
    tag_3d_points = np.array([
        [-tag_size/2, -tag_size/2, 0.0],
        [tag_size/2, -tag_size/2, 0.0],
        [tag_size/2, tag_size/2, 0.0],
        [-tag_size/2, tag_size/2, 0.0]
    ])

    imgPointsArr.append(r.corners)
    objPointsArr.append(tag_3d_points)

    # Estimate the pose of the tag
    _, rvec, tvec = cv2.solvePnP(tag_3d_points, r.corners, mtx, distCoeffs=dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    rotation_matrix, _ = cv2.Rodrigues(rvec)

    print("Rotation Matrix:")
    print(rotation_matrix)
    print("Translation Vector:")
    print(tvec)

# Display the image with detected tags
cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
