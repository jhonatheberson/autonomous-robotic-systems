# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import math
import numpy as np


point_init = (-0.0000,-0.0000, 0.0000)
point_end = (1.5250e+00, -1.4750e+00, 0.0000)
#Ganhos do controlador
k_theta = 0.1;
k_l = 0.1;
#Robot model
d = 0.21 #wheel axis distance
r_w = 0.0625 #wheel radius

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import ctypes


#transform from image frame to vrep frame
def transform_points_from_image2real (points):
    if points.ndim < 2:
        flipped = np.flipud(points)
    else:
        flipped = np.fliplr(points)
    scale = 5/445
    points2send = (flipped*-scale) + np.array([2.0555+0.75280899, -2.0500+4.96629213])
    return points2send

def send_path_4_drawing(path, sleep_time = 0.07):
    #the bigger the sleep time the more accurate the points are placed but you have to be very patient :D
    for i in path:
        packedData=sim.simxPackFloats(i.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
        returnCode=sim.simxWriteStringStream(clientID, "path_coord", raw_bytes, sim.simx_opmode_oneshot)
        time.sleep(sleep_time)


def polinomio(point_init, point_end):
    sigma = 1
    deltaX = point_end[0] - point_init[0]
    deltaY = point_end[1] - point_init[1]
    alfa_init = math.tan(point_init[2])
    alfa_end = math.tan(point_end[2])

    if point_init[2] >= ((math.pi/2) - sigma) and point_init[2] <= ((math.pi/2) + sigma) and point_end[2] >= ((math.pi/2) - sigma) and point_end[2] >= ((math.pi/2) + sigma):
        print('i')
        b1 = deltaY
        b2 = 0
        a0 = point_init[0]
        a1 = 0
        a2 = 3*deltaX
        a3 = -2*deltaX
        b0 = point_init[1]
        b3 = deltaY-b1-b2

    elif point_init[2] >= ((math.pi/2) - sigma) and point_init[2] <= ((math.pi/2) + sigma):
        print('ii')
        a3 = -(deltaX/2)
        b3 = 1
        a0 = point_init[0]
        a1 = 0
        a2 = deltaX-a3
        b0 = point_init[1]
        b1 = 2*(deltaY-alfa_end*deltaX) - alfa_end*a3 + b3
        b3 = (2*alfa_end*deltaX-deltaY) + alfa_end*a3 - 2*b3
    elif point_end[2] >= ((math.pi/2) - sigma) and point_init[2] <= ((math.pi/2) + sigma):
        print('iii')
        a1 = 3*(deltaX/2)
        b2 = 1
        a0 = point_init[0]
        a2 = 3*deltaX - 2*a1
        a3 = a1 - 2*deltaX
        b0 = point_init[1]
        b1 = alfa_init*a1
        b3 = deltaY - alfa_init*a1 - b2
    else:
        print('iv')
        a1 = deltaX
        a2 = 0
        a0 = point_init[0]
        a3 = deltaX - a1 - a2
        b0 = point_init[1]
        b1 = alfa_init*a1
        b2 = 3*(deltaY-alfa_end*deltaX) + 2*(alfa_end-alfa_init)*a1 + alfa_end*a2
        b3 = 3*alfa_end*deltaX - 2*deltaY - (2*alfa_end-alfa_init)*a1 -  alfa_end*a2

    
    
    result = []
    orien = []
    x  = np.arange(0,1,0.01)
    for i in range(len(x)):
        fx = a0 + a1*x[i] + a2*x[i]**2 + a3*x[i]**3
        fy = b0 + b1*x[i] + b2*x[i]**2 + b3*x[i]**3
        if fx != 0:
            orientation = np.arctan(float(fy/fx))
        else: 
            orientation = 0
        position = np.array((fx, fy, 0)) 
        result.append(position)
                      
    return (result, orientation)

def controlador_posicao(delta_x, delta_y, orientacao):
    Kr = 0.6 
    kdelta = 0.1
    ksigma = 0.1
    R = (delta_x**2+delta_y**2)**(1/2)
    delta = np.arctan((delta_y/delta_x))
    sigma = delta + orientacao
    v = Kr*R*math.cos(delta)
    w = kdelta*delta + (Kr*(delta + ksigma*sigma)*(math.sin(delta)*math.cos(delta)))/delta
    return v,w


print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)


    

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    err_code,l_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Direito", sim.simx_opmode_blocking)
    err_code,r_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Esquerdo", sim.simx_opmode_blocking)

    err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
    err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
    
    err_code,destino_robo = sim.simxGetObjectHandle(clientID,"Destino", sim.simx_opmode_blocking)
    err_code,destino_robo = sim.simxGetObjectPosition(clientID,destino_robo,-1, sim.simx_opmode_blocking)
    
    err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
    err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
    print ('posicao robo:', posicao_robo)
    print ('Destino robo:',destino_robo)
    print ('orientação do robo:', orientacao_robo)


    
    caminho, orientation = polinomio(point_init,point_end)
    send_path_4_drawing(caminho, 0.05)

    #############################
    #teste controle estabilizante
    while True:
      err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
      err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
      
      err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
      err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
      
      theta_robo = orientacao_robo[2]
      print("theta_robo: ", theta_robo)
      
      if (theta_robo > math.pi):
        theta_robo = theta_robo + 2*math.pi
      elif (theta_robo < -math.pi):
        theta_robo = theta_robo - 2*math.pi
      #calculo dos delta x,y
      delta_x = point_end[0] - posicao_robo[0]
      delta_y = point_end[1] - posicao_robo[1]
      
      
      #if (theta_robo < 0 ):
      #  theta_robo = theta_robo + math.pi/2
      
      #theta_estrela/referencial
      theta_ref = np.arctan(delta_y/delta_x)
      print("theta_ref: ", theta_ref)
      #calculo do delta_l/referencial, delta_theta
      delta_l_ref = np.sqrt((delta_x)**2 + (delta_y**2))
      delta_theta = theta_ref - theta_robo
      
      if (delta_theta > math.pi):
        delta_theta = delta_theta - 2*math.pi
      elif (delta_theta < -math.pi):
        delta_theta = delta_theta + 2*math.pi
      #calculo do delta_l
      
      delta_l = delta_l_ref*np.cos(delta_theta)

      v = k_l*delta_l
      w = k_theta*delta_theta
      #v, w = controlador_posicao(delta_x, delta_y, delta_theta)
      wd = (v/r_w) + (d/(2*r_w))*w
      we = (v/r_w) - (d/(2*r_w))*w
      
      err_code = sim.simxSetJointTargetVelocity(clientID,l_motor_handle,we,sim.simx_opmode_streaming)
      err_code = sim.simxSetJointTargetVelocity(clientID,r_motor_handle,wd,sim.simx_opmode_streaming)
      
      if delta_l_ref <= 0.1:
        break

    #teste controle estabilizante
    #############################

    err_code = sim.simxSetJointTargetVelocity(clientID,l_motor_handle,0,sim.simx_opmode_streaming)
    err_code = sim.simxSetJointTargetVelocity(clientID,r_motor_handle,0,sim.simx_opmode_streaming)

    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
