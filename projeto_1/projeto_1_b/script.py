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
point_end = (1.5250e+00, 1.8000e+00, 0.0000)




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

#transform from vrep frame to robot frame
#def transform2robot_frame(pos, point, theta):
    #pos = np.asarray(pos)
    #point = np.asarray(point)
    #T_matrix = np.array([[np.sin(theta), np.cos(theta)],[np.cos(theta), -1*np.sin(theta)],])
    #trans = point-pos
    #if trans.ndim >= 2:
        #trans = trans.T
        #point_t = np.dot(T_matrix, trans).T
    #else:
        #point_t = np
    #return point_t

#Robot model
d = 0.21 #wheel axis distance
r_w = 0.0625 #wheel radius


def seguidor_caminho():
    
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
    k_theta = 0.6
    k_l = 0.1

    #Polinomio interpolador no matlab
    #[X, Y, Theta, a, b, Mat] = poli_cubic([-2,-2,0],[2,2,45],0.01)
    
    #parametros do robo
    v = 0.5
    d = 0.21 #distancia do eixo entre as rodas
    rd = 0.0625 #raio roda direita
    re = 0.0625 #raio roda esquerda

    caminho,orientacao = polinomio(point_init, point_end)

    k = 0
    z = np.arange(0,1,0.01) 
    for k in range(len(z)): #Enquanto o robo se movimenta
        #pegar posicao e orientacao do robo sempre atualizando
        err_code,posicao_robo = sim.simxGetObjectPosition(clientID,carro,-1, sim.simx_opmode_blocking)
        err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_streaming)

        #calculo da menor distância do robô a curva
        #[ponto_curva, Delta_l, lbda] = distance2curve(Mat(:,1:2),[posicao_robo(1),posicao_robo(2)],'linear');

        #ponto da curva
        distancia = []
        #j = 0
        #for j in range(len(caminho[0])):
        for i in caminho:
            distancia.append(np.sqrt(pow((posicao_robo[0] - i[0]),2) + pow((posicao_robo[1] - i[1]),2)))
                #if j == 0:
                    #ponto_curva = caminho [j]
                #elif distancia[j]<distancia[j-1]:
                    #ponto_curva = caminho[j]
        print ('distancia min:', np.min(distancia))
        ponto_curva_x = np.min(distancia) + posicao_robo[0]
        ponto_curva_y = np.min(distancia) + posicao_robo[1]
        ponto_curva = (ponto_curva_x,ponto_curva_y)

        #raio de giro
        a,b = coeficientes(point_init,point_end)
        dx = a[1] +2*a[2]*k + 3*a[3]*(k**2)
        dy = b[1] + 2*b[2]*k + 3*b[3]*(k**2)
        d2x = 2*a[2] + 6*a[3]*k
        d2y = 2*b[2] + 6*b[3]*k
        r = ((((pow(dx,2)))+(pow((pow(dy,2)),(1.5))))/((d2y*dx)-(d2x*dy)))             
        k = (1/r)

        orientation = np.arctan((b[1] + 2*b[2]*k + 3*b[3]*((pow(k,2))))/(a[1] + 2*a[2]*k + 3*a[3]*(pow(k,2))))

        #delta theta
        print ('orientacao robo:',orientacao_robo)
        Theta_robo = orientacao_robo[2] #+ math.pi/2  
        theta_SF = orientation
        Delta_theta = Theta_robo - theta_SF

        #delta_l é a distancia
        Delta_l = np.min(distancia)

        #Garantir o sinal correto de Delta L 
        theta_posicao_robo = np.arctan2(posicao_robo[1],posicao_robo[0]); #angulo de posicao do robo
        theta_ref = np.arctan2(ponto_curva[1],ponto_curva[0]); #angulo da curva que está mais próxima ao robo
         
        if(theta_ref>theta_posicao_robo): #Sinal do delta L
            Delta_l = -Delta_l

        #Controle
        u = -(k_theta*Delta_theta + (k_l*Delta_l*v*np.sin(Delta_theta)/Delta_theta))
         
        #Velocidade Angular
        w = u + ((k*v*np.cos(Delta_theta))/(1-(k*Delta_l)))
         
        #Velocidade das juntas
        wd = (v/rd) + (d/(2*rd))*w
        we = (v/re) - (d/(2*re))*w
         
        sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,we,sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(clientID,motor_direito,wd,sim.simx_opmode_blocking)

        if k==1:           
            returnCode=sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,0,sim.simx_opmode_streaming)
            returnCode=sim.simxSetJointTargetVelocity(clientID,motor_direito,0,sim.simx_opmode_streaming) 
    #k += 0.01  

    #Plot do erro Delta l
    #figure(2)
    #plot(ErroL)
    #grid on;
    #ylabel('Delta L')
    #xlabel('Interação')

    returnCode=sim.simxSetJointTargetVelocity(clientID,motor_esquerdo,0,sim.simx_opmode_streaming)
    returnCode=sim.simxSetJointTargetVelocity(clientID,motor_direito,0,sim.simx_opmode_streaming)

    sim.simxFinish(-1)
 

   
    
###########IAGO##################
#def seguidor_trajetoria(objeto, caminho, posicao_atual):
#        err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
 #       err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
  #      err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
   #     err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
    #    print ('Pocicao inicial:', posicao_robo)
     #   print ('Orientacao inicial:', orientacao_robo)
#
 #       while posicao_atual != destino_robo:
  #          
   #     err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
    #    err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
     #   err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
      #  err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
       # print ('Pocicao inicial:', posicao_robo)
        #print ('Orientacao inicial:', orientacao_robo)

    
  #  return 0

#################################
#def controlador_posicao(coordenadas, orientacao):
    #kr = kdelta = ksigma = 1
    #R = (coordenadas[0]**2+coordenadas[1]**2)**(1/2)
    #delta = np.arctan((coordenadas[1]/coordenadas[0]))
    #sigma = delta + orientacao
    #v = Kr*R*math.cos(delta)
    #w = kdelta*delta + (kr*(delta + ksigma*sigma)*(math.sen(delta)*math.cos(delta)))/delta
    #return v,w


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
    #startTime=time.time()
    #err_code,l_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Direito", sim.simx_opmode_blocking)
    #err_code,r_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Esquerdo", sim.simx_opmode_blocking)

    #err_code = sim.simxSetJointTargetVelocity(clientID,l_motor_handle,0.0,sim.simx_opmode_streaming)
    #err_code = sim.simxSetJointTargetVelocity(clientID,r_motor_handle,0.0,sim.simx_opmode_streaming)

    #err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
    #err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
    #err_code,destino_robo = sim.simxGetObjectHandle(clientID,"Destino", sim.simx_opmode_blocking)
    #err_code,destino_robo = sim.simxGetObjectPosition(clientID,destino_robo,-1, sim.simx_opmode_blocking)
    #err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
    #err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
    #print ('posicao robo:', posicao_robo)
    #print ('Destino robo:',destino_robo)
    #print ('orientação do robo:', orientacao_robo)


    

    #############################
    #teste controle estabilizante
    caminho, orientation = polinomio(point_init,point_end)
    send_path_4_drawing(caminho, 0.05)
    #v, w = controlador_posicao(posicao_robo, orientacao_robo)

    #teste controle estabilizante
    #############################

    seguidor_caminho()
    
    print ('Finalizado seguidor de caminho')


    #sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    # while time.time()-startTime < 5:
    #     returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
    #     if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    #         print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
    #     time.sleep(0.005)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    #sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    #sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
