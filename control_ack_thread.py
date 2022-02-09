import vrep # access all the VREP elements
import numpy as np
import time
import math
import matplotlib.pyplot as mlp
import threading 
from PIL import Image as I
import array
import cv2

stop_sign = False

#Function that filter only red color (used as substitute for stop sign) and see the distance based os the percentage of red pixels in frame
def track_red_object(img):
    # blur_img = cv2.GaussianBlur(img, (5,5),0)

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    low_red = np.array([0, 100, 100])
    high_red = np.array([179, 255, 255])
    red_mask = cv2.inRange(hsv_img, low_red, high_red)

    n_white_pix = np.sum(red_mask == 255)

    num_rows, num_cols = red_mask.shape
    total = num_cols * num_rows

    percent = (n_white_pix/total) * 100

    return percent

#Function that will run as thread to control all cameras used to detection
def camera_control(clientID):
    print("Runing camera thread")

    res, vs0 = vrep.simxGetObjectHandle(clientID, 'v0', vrep.simx_opmode_oneshot_wait)
    res, vs1 = vrep.simxGetObjectHandle(clientID0, 'v1', vrep.simx_opmode_oneshot_wait)
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, vs0, 0, vrep.simx_opmode_streaming)
    err, resolution2, image2 = vrep.simxGetVisionSensorImage(clientID, vs1, 0, vrep.simx_opmode_streaming)
    time.sleep(1)

    cv2.namedWindow("Right")
    cv2.namedWindow("Left")

    while vrep.simxGetConnectionId(clientID) != -1:
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, vs0, 0, vrep.simx_opmode_buffer)
        err, resolution2, image2 = vrep.simxGetVisionSensorImage(clientID, vs1, 0, vrep.simx_opmode_buffer)

        global stop_sign

        if err == vrep.simx_return_ok:
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            img2 = np.array(image2,dtype=np.uint8)
            img2.resize([resolution2[1],resolution2[0],3])
            try:
                cv2.imshow('Right',img)
                cv2.imshow('Left',img2)
            except:
                print('Camera crashed')
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            stop_indicator_right = track_red_object(img)
            stop_indicator_left= track_red_object(img2)

            if (stop_indicator_right >= 50.0):
                print("Stop sign on right side")
                stop_sign = True
                time.sleep(2)
                stop_sign = False
                time.sleep(1)
            if (stop_indicator_left >= 50.0):
                print("Stop sign on left side")
                stop_sign = True
                time.sleep(2)
                stop_sign = False
                time.sleep(1)

        elif err == vrep.simx_return_novalue_flag:
            pass
        else:
            print (err)


#Function that will run as thread to control all of basics movements of vehicle, plus the collision detection
def joint_control(clientID):

    print("Runing joint thread")

    speed = 1
    desiredSteeringAngle = 0
    count = 0
    d=0.755 # 2*d=distance between left and right wheels
    l=2.5772 # l=distance between front and read wheels

    err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"nakedCar_motorLeft", vrep.simx_opmode_blocking)
    err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"nakedCar_motorRight", vrep.simx_opmode_blocking)
    err_code,steeringLeft = vrep.simxGetObjectHandle(clientID,"nakedCar_steeringLeft", vrep.simx_opmode_blocking)
    err_code,steeringRight = vrep.simxGetObjectHandle(clientID,"nakedCar_steeringRight", vrep.simx_opmode_blocking)

    err_code,ds = vrep.simxGetObjectHandle(clientID,"ps",  vrep.simx_opmode_blocking)

    res,v1 = vrep.simxGetObjectHandle(clientID, 'left_light', vrep.simx_opmode_oneshot_wait)   
    res,v2 = vrep.simxGetObjectHandle(clientID, 'right_light', vrep.simx_opmode_oneshot_wait)

    err_code,cilinder = vrep.simxGetObjectHandle(clientID, 'cyl', vrep.simx_opmode_oneshot_wait)

    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetPosition(clientID, steeringLeft, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetPosition(clientID, steeringRight, 0, vrep.simx_opmode_streaming)

    resultR,slaR,dataR = vrep.simxReadVisionSensor(clientID, v2, vrep.simx_opmode_streaming)
    resultL,slaL,dataL = vrep.simxReadVisionSensor(clientID, v1, vrep.simx_opmode_streaming)

    errFS,slaFS,dataFS,dhFS,dvFS = vrep.simxReadProximitySensor(clientID, ds, vrep.simx_opmode_streaming)

    global stop_sign

    while vrep.simxGetConnectionId(clientID) != -1:

        count = count + 1

        errR,slaR,dataR = vrep.simxReadVisionSensor(clientID, v2, vrep.simx_opmode_buffer)
        errL,slaL,dataL = vrep.simxReadVisionSensor(clientID, v1, vrep.simx_opmode_buffer)

        errFS,slaFS,dataFS,dhFS,dvFS = vrep.simxReadProximitySensor(clientID, ds, vrep.simx_opmode_buffer)

        dist = np.linalg.norm(dataFS)

        # print(stop_sign)

        if (stop_sign == True):
            speed = 0.001
            err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
            err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)
        else:
            if (dist < 0.2) and (dist > 0.01):
                if (speed >= 0.1):
                    speed = speed - 0.01
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)
                else:
                    speed = 0.01
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)

                    time.sleep(1)

                    err_code = vrep.simxSetObjectPosition(clientID,cilinder,-1,[-10.0,-10.0,-10.0],vrep.simx_opmode_oneshot)
            else:
                if (speed < 1):
                    while (speed < 1):
                        speed = speed + 0.01
                        err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
                        err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)
                        
                    speed = 1
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, speed, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, speed, vrep.simx_opmode_streaming)   
        if (errL == vrep.simx_return_ok):

            try:
                if count > 1:
                    if dataR[0][11] < 0.4:
                        desiredSteeringAngle = -(45*math.pi)/180
                    if dataL[0][11] < 0.4:
                        desiredSteeringAngle = (45*math.pi)/180
                    if (dataR[0][11] > 0.4) and (dataL[0][11] > 0.4):
                        desiredSteeringAngle = 0  

                if desiredSteeringAngle != 0:
                    steeringAngleLeft=math.atan(l/(-d+l/math.tan(desiredSteeringAngle)))
                    steeringAngleRight=math.atan(l/(d+l/math.tan(desiredSteeringAngle)))
                else:
                    steeringAngleLeft = 0
                    steeringAngleRight = 0

                err_code = vrep.simxSetJointTargetPosition(clientID, steeringLeft, steeringAngleLeft, vrep.simx_opmode_streaming)
                err_code = vrep.simxSetJointTargetPosition(clientID, steeringRight, steeringAngleRight, vrep.simx_opmode_streaming)
            except:
                print("Something broke")

        elif (errL == vrep.simx_return_novalue_flag):
            pass

        else:
            break


if __name__ == "__main__": 
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID0=vrep.simxStart('127.0.0.1', 19999, True, True, 5000,5) # start a connection

    if clientID0!=-1:
        #Start threads
        threading.Thread(target=joint_control, args=(clientID0,)).start()
        threading.Thread(target=camera_control, args=(clientID0,)).start()
            

    else:
        print("Not connected to remote API server")
        vrep.simxFinish(clientID0)

