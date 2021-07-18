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
    d=0.755 # 2*d=distance between left and right wheels
    l=2.5772 # l=distance between front and read wheels

    err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"nakedCar_motorLeft", vrep.simx_opmode_blocking)
    err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"nakedCar_motorRight", vrep.simx_opmode_blocking)
    err_code,steeringLeft = vrep.simxGetObjectHandle(clientID,"nakedCar_steeringLeft", vrep.simx_opmode_blocking)
    err_code,steeringRight = vrep.simxGetObjectHandle(clientID,"nakedCar_steeringRight", vrep.simx_opmode_blocking)

    res, v1 = vrep.simxGetObjectHandle(clientID, 'left_light', vrep.simx_opmode_oneshot_wait)   
    res, v2 = vrep.simxGetObjectHandle(clientID, 'right_light', vrep.simx_opmode_oneshot_wait)
    res, v3 = vrep.simxGetObjectHandle(clientID, 'middle_light', vrep.simx_opmode_oneshot_wait)

    err_code = vrep.simxSetJointTargetVelocity(clientID, l_motor_handle, 1, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID, r_motor_handle, 1, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetPosition(clientID, steeringLeft, 0, vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetPosition(clientID, steeringRight, 0, vrep.simx_opmode_streaming)

    print(err_code)

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
            print(err_code)
            err_code = vrep.simxSetJointTargetPosition(clientID, steeringRight, steeringAngleRight, vrep.simx_opmode_streaming)
            print(err_code)

        elif (errL == vrep.simx_return_novalue_flag):
            pass

        else:
            break
else:
    print("Not connected to remote API server")
    vrep.simxFinish(clientID)