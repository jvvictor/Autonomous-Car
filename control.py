import vrep # access all the VREP elements
import numpy as np
import time
import math
import matplotlib.pyplot as mlp

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1', 19999, True, True, 5000,5) # start a   connection

if clientID!=-1:

    print ("Connected to remote API server")

    count = 0
    err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_leftMotor", vrep.simx_opmode_blocking)
    err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_rightMotor", vrep.simx_opmode_blocking)

    res, v1 = vrep.simxGetObjectHandle(clientID, 'left_light', vrep.simx_opmode_oneshot_wait)   
    res, v2 = vrep.simxGetObjectHandle(clientID, 'right_light', vrep.simx_opmode_oneshot_wait)
    res, v3 = vrep.simxGetObjectHandle(clientID, 'middle_light', vrep.simx_opmode_oneshot_wait)

    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle,2, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, 2, vrep.simx_opmode_streaming)

    resultR,slaR,dataR = vrep.simxReadVisionSensor(clientID, v2, vrep.simx_opmode_streaming)
    resultL,slaL,dataL = vrep.simxReadVisionSensor(clientID, v1, vrep.simx_opmode_streaming)
    resultM,slaM,dataM = vrep.simxReadVisionSensor(clientID, v3, vrep.simx_opmode_streaming)

    while vrep.simxGetConnectionId(clientID) != -1:
        
        count = count + 1

        errR,slaR,dataR = vrep.simxReadVisionSensor(clientID, v2, vrep.simx_opmode_buffer)
        errL,slaL,dataL = vrep.simxReadVisionSensor(clientID, v1, vrep.simx_opmode_buffer)
        errM,slaM,dataM = vrep.simxReadVisionSensor(clientID, v3, vrep.simx_opmode_buffer)

        if (errL == vrep.simx_return_ok):
            if count > 1:
                if dataR[0][11] < 0.4:
                    desiredangle = (5*math.pi)/180
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, 1, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, desiredangle, vrep.simx_opmode_streaming)
                if dataL[0][11] < 0.4:
                    desiredangle = (5*math.pi)/180
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, 1, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, desiredangle, vrep.simx_opmode_streaming)
                if (dataR[0][11] > 0.4) and (dataL[0][11] > 0.4):
                    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, 2, vrep.simx_opmode_streaming)
                    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, 2, vrep.simx_opmode_streaming)    

        elif (errL == vrep.simx_return_novalue_flag):
            pass

        else:
            break
else:
    print("Not connected to remote API server")
    vrep.simxFinish(clientID)
