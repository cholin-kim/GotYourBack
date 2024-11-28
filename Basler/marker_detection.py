import numpy as np
import cv2
import cv2.aruco as aruco

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = cv2.aruco.generateImageMarker(dictionary, 23, 200, markerImage, borderBits=1)

parameters =  aruco.DetectorParameters()

k = np.array([[5.63826892e+03, 0.00000000e+00, 2.39401413e+02],
 [0.00000000e+00, 4.46636598e+03, 3.17556975e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]], dtype=np.float64)
d = np.zeros((1, 5))

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners)
        for i in range(len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, k, d)
            cv2.drawFrameAxes(frame, k, d, rvec, tvec, 0.01)
    else:
        cv2.putText(frame, "No markers detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    cv2.imshow('frame', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break