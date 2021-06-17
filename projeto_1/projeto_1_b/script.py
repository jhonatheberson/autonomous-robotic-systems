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
point_end = (+1.5250e+00, -1.4750e+00, +5.0000e-02)
#point_end = (1.5250e+00, 1.8000e+00, 0.0000)




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


def send_path_4_drawing(path, sleep_time = 0.07):
    #the bigger the sleep time the more accurate the points are placed but you have to be very patient :D
    for i in path:
        #point2send = transform_points_from_image2real(i)
        #print(point2send)
        #print(type(point2send))
        packedData=sim.simxPackFloats(i.flatten())
        #print(packedData)
        #print(type(packedData))
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData)
        #print(raw_bytes)
        #print(type(raw_bytes))
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

def coeficientes (origem,destino):
    sigma=1
    deltaX = origem[0] - destino[0]
    deltaY = origem[1] - destino[1]
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

    a = [a0,a1,a2,a3]
    b = [b0,b1,b2,b3]
    
    x  = np.arange(0,1,0.01)
    for i in range(len(x)):
        fx = a0 + a1*x[i], a2*x[i]**2 + a3*x[i]**3
        fy = b0 + b1*x[i] + b2*x[i]**2 + b3*x[i]**3
        #if fx != 0:
            #orientation = np.arctan(float(fy/fx))
        #else: 
           # orientation = 0
                      
    return (a,b)

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


    

    #############################
    #teste controle estabilizante
    caminho, orientation = polinomio(point_init,point_end)
    send_path_4_drawing(caminho, 0.05)
    #v, w = controlador_posicao(posicao_robo, orientacao_robo)

    #teste controle estabilizante
    #############################
    #Fazendo os handles
    err_code,motor_direito = sim.simxGetObjectHandle(clientID,"Motor_Direito", sim.simx_opmode_blocking)
    err_code,motor_esquerdo = sim.simxGetObjectHandle(clientID,"Motor_Esquerdo", sim.simx_opmode_blocking)
    err_code,carro = sim.simxGetObjectHandle(clientID,"Carro", sim.simx_opmode_blocking)
    
    #Zerando as velocidades das rodas
    err_code = sim.simxSetJointTargetVelocity(clientID,motor_direito,0,sim.simx_opmode_streaming)
    err_code = sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,0,sim.simx_opmode_streaming)
    
    #Posicao e orientacao do robo
    err_code,posicao_robo = sim.simxGetObjectPosition(clientID,carro,-1, sim.simx_opmode_blocking)
    er_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_streaming)
    
    #Ganhos do controlador
    k_theta = 2.0
    k_l = 0.1
    
    
    #parametros do robo
    i = 1
    v = 0.5
    d = 0.21 #distancia do eixo entre as rodas
    rd = 0.0625 #raio roda direita
    re = 0.0625 #raio roda esquerda

    caminho,orientacao = polinomio(point_init, point_end)
    lamb = 0
    while True:
      #Posicao e orientacao do robo
      err_code,posicao_robo = sim.simxGetObjectPosition(clientID,carro,-1, sim.simx_opmode_blocking)
      er_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_streaming)
      
      theta_robo = orientacao_robo[2] + math.pi/2
      
      #raio de giro
      a,b = coeficientes(point_init,point_end)
      dx = a[1] +2*a[2]*lamb + 3*a[3]*(lamb**2)
      dy = b[1] + 2*b[2]*lamb + 3*b[3]*(lamb**2)
      d2x = 2*a[2] + 6*a[3]*lamb
      d2y = 2*b[2] + 6*b[3]*lamb
      r = ((((dx**2)+(dy**2))**1.5)/((d2y*dx)-(d2x*dy)))             
      k = (1/r)
      
      #delta theta
      theta_SF = math.atan((b[1] + 2*b[2]*lamb + 3*b[3]*lamb**2)/(a[1] + 2*a[2]*lamb + 3*a[3]*lamb**2))
      Delta_theta = theta_robo - theta_SF
      
      #garantir o sinal correto de delta L
      theta_posicao_robo = math.atan2(posicao_robo[1],posicao_robo[0])
      delta_L = np.linalg.norm(posicao_robo-caminho[0])
      ponto_curva = caminho[0]
      for i in range(len(caminho)-1):
        distance = np.linalg.norm(posicao_robo-caminho[i+1])
        if(delta_L > distance):
          delta_L = distance
          ponto_curva = caminho[i+1]
      theta_ref = math.atan2(ponto_curva[1],ponto_curva[0])
      
      if (theta_ref > theta_posicao_robo):
        delta_L = -delta_L
        
      i = i +1
      
      u = -(k_theta*Delta_theta + (k_l*delta_L*v*math.sin(Delta_theta)/Delta_theta))
      
      w = u + ((k*v*math.cos(Delta_theta))/(1-(k*delta_L)))
      
      wd = (v/rd) + (d/(2*rd))*w
      we = (v/re) - (d/(2*re))*w
      
      err_code = sim.simxSetJointTargetVelocity(clientID,motor_direito,wd,sim.simx_opmode_streaming)
      err_code = sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,we,sim.simx_opmode_streaming)
      print('lamb :', lamb)
      if (lamb >= 1):
        err_code = sim.simxSetJointTargetVelocity(clientID,motor_direito,0.0,sim.simx_opmode_streaming)
        err_code = sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,0.0,sim.simx_opmode_streaming)
        break
      
      lamb = lamb + 0.01
    
    err_code = sim.simxSetJointTargetVelocity(clientID,motor_direito,0.0,sim.simx_opmode_streaming)
    err_code = sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,0.0,sim.simx_opmode_streaming)
    print ('Finalizado seguidor de caminho')

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
