import vrep
import time
import cv2
import numpy as np
import imutils

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

red_lower = np.array([-10, 50, 50])
red_upper = np.array([10, 255, 255])

if clientID != -1:
    print('Connected to remote API server')
    print('Vision Sensor object handling')
    res, v1 = vrep.simxGetObjectHandle(clientID, 'left_light', vrep.simx_opmode_oneshot_wait)
    print('Getting first image')
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    while vrep.simxGetConnectionId(clientID) != -1:
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            print("image OK!!!")

            img = np.array(image, dtype=np.uint8)
            img.resize([resolution[0], resolution[1], 3])
            img = imutils.rotate_bound(img, 180)
            
            cv2.startWindowThread()
            cv2.namedWindow("image")
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == vrep.simx_return_novalue_flag:
            print("no image yet")
            pass
        else:
            print(err)
else:
    print("Failed to connect to remote API Server")
    vrep.simxFinish(clientID)

cv2.destroyAllWindows()