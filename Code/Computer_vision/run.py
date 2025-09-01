import cv2
import numpy as np
import src.DoDecahedronUtils as dodecapen
import src.Tracker as tracker
import time

# TODO: Call Camera
cap = cv2.VideoCapture(0)

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
print(frame_height, frame_width)
cap.set(cv2.CAP_PROP_SETTINGS, 1)

post = 2

cv2.namedWindow('RGB-D Live Stream', cv2.WINDOW_NORMAL)
cv2.resizeWindow("RGB-D Live Stream", frame_width, frame_height)
cv2.moveWindow("RGB-D Live Stream", 20, 20)

# TODO: Camera matrix Calibration   params.mtx, params.dist
# https://docs.opencv.org/4.x/d/dbb/tutorial_py_calibration.html
color_cam_matrix = np.load("./camera_matrix/color_cam_matrix.npy")
color_cam_dist = np.load("./camera_matrix/color_cam_dist.npy")

# TODO: ArucoUtils.py  ddc_parameters    ddc_txt_data
ddc_text_data = dodecapen.txt_data()
ddc_params = dodecapen.parameters()

# TODO: 4X4   X   4X1  =  4X1
tip_loc_cent = np.array([0.15100563, 137.52252061, -82.07403558, 1]).reshape(4, 1)

while(True):
    ret, rgb_image = cap.read()
    if not ret:
        break
    depth_image = None
    if rgb_image is None:
        time.sleep(0.1)
        print("No image")
        continue


    object_pose = tracker.object_tracking(rgb_image, ddc_params, ddc_text_data, post)


    if object_pose is not None:
        print(object_pose)
        cv2.drawFrameAxes(rgb_image, ddc_params.mtx, ddc_params.dist,
                          object_pose[:, :3],
                          object_pose[0, 3:].reshape(1, 3), 20)
        
        # Convert the object's pose into a 4x4 transformation matrix
        T_cam_obj = dodecapen.RodriguesToTransf(object_pose)

        # Calculate the pentip's position in the camera frame
        tip_loc_cam = np.dot(T_cam_obj, tip_loc_cent)

        # Convert the pentip's position to a 3-element vector
        tip_tvec = tip_loc_cam[:3, :].reshape(1, 3)

        # Use a dummy rotation vector (all zeros) for the pentip, as we only care about its position
        # A rotation of 0 means the coordinate system will be aligned with the camera's axes
        dummy_rvec = np.zeros((1, 3))

        # Draw a second coordinate system at the estimated pentip position
        cv2.drawFrameAxes(rgb_image, ddc_params.mtx, ddc_params.dist,
                          dummy_rvec,
                          tip_tvec, 20)


    cv2.imshow('RGB-D Live Stream', rgb_image)

    if cv2.waitKey(1) == ord("q"):
        break

cv2.destroyAllWindows()


