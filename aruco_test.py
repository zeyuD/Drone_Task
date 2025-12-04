import aruco_markers as am
import numpy as np
import cv2
import time

def sleep_for_realtime_playback(started, last_frame_time):
    if started == 0:
        last_frame_time = time.time()
        started = 1
        print("Video start time:", last_frame_time)
    else:
        elapsed = time.time() - last_frame_time
        if elapsed < vid_frame_time:
            print("Sleeping for:", vid_frame_time - elapsed)
            time.sleep(vid_frame_time - elapsed)
    return started, last_frame_time

camera_index = 0
marker_length = 0.1  # Size of the ArUCo tag in meters
dict_name = "DICT_4X4_50"
marker_ids = [0, 1, 2, 3]  # Identifier of the marker

detector = am.load_detector(dict_name)
# camera = am.cvCamera(camera_index)
# pose_detector = am.SingleMarkerPoseEstimation(camera, marker_length)
vid_file = "aruco.mp4"
vid = cv2.VideoCapture(vid_file)
vid_fps = vid.get(cv2.CAP_PROP_FPS)
vid_frame_time = 1.0 / vid_fps
print("Video FPS:", vid_fps)
print("Frame time (s):", vid_frame_time)

vid_to_save = cv2.VideoWriter('aruco_detected.avi', 
                             cv2.VideoWriter_fourcc(*'XVID'), 
                             vid_fps, 
                             (int(vid.get(cv2.CAP_PROP_FRAME_WIDTH)), int(vid.get(cv2.CAP_PROP_FRAME_HEIGHT))))

started = 0
while True:
    last_frame_time = time.time()
    # started, last_frame_time = sleep_for_realtime_playback(started, last_frame_time)
    ret, img = vid.read()
    if not ret:
        break
    # print("Image shape:", img.shape)

    corners, ids, _ = detector.detectMarkers(img)
    for i, c in enumerate(corners):

        # print("Detected marker ID:", ids[i][0])
        # if ids[i][0] in marker_ids:
        #     print("Marker corners:", c.reshape(-1, 2))

        if ids[i][0] in marker_ids: # Only process specified marker IDs

            # Draw detected marker
            img = cv2.aruco.drawDetectedMarkers(img, (c,), np.array([ids[i]]))

            # print("Image shape:", img.shape)

            # Estimate marker pose
            # pose = pose_detector.estimate_marker_pose(c)
            
        # cv2.imshow("Aruco Marker Detection", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    vid_to_save.write(img)

vid.release()
vid_to_save.release()
cv2.destroyAllWindows()