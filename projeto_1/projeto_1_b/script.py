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

#transform from vrep frame to robot frame
def transform2robot_frame(pos, point, theta):
    pos = np.asarray(pos)
    point = np.asarray(point)
    T_matrix = np.array([[np.sin(theta), np.cos(theta)],[np.cos(theta), -1*np.sin(theta)],])
    trans = point-pos
    if trans.ndim >= 2:
        trans = trans.T
        point_t = np.dot(T_matrix, trans).T
    else:
        point_t = np
    return point_t

#Robot model
d = 0.21 #wheel axis distance
r_w = 0.0625 #wheel radius

def car_robot_model(v_des, omega_des):
    v_r = (v_des+d*omega_des)
    v_l = (v_des-d*omega_des)
    omega_right = v_r/r_w
    omega_left = v_l/r_w
    return omega_right, omega_left

def is_near(robot_center, point, dist_thresh = 0.25):
    dist = np.sqrt((robot_center[0]-point[0])**2 + (robot_center[1]-point[1])**2)
    return dist<=dist_thresh

def get_distance(points1, points2):
    return np.sqrt(np.sum(np.square(points1 - points2), axis=1))

class pid():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0.0
        self.error_old = 0.0
        self.error_sum = 0.0
        self.d_error = self.error - self.error_old
    def control(self,error):
        self.error = error
        self.error_sum += error
        self.d_error = self.error - self.error_old
        P = self.kp*self.error
        I = self.ki*self.error_sum
        D = self.kd*self.d_error
        self.error_old 

def seguidor_caminho(origem,destino,caminho):
    lad = 0.5 #look ahead distance
    dt = 0.0
    count = 0
    om_sp = 0
    d_controller   = pid(kp=0.5, ki=0, kd=0)
    omega_controller = pid(0.5, 0., 0.)
    err_code,look_ahead_sphere = sim.simxGetObjectHandle(clientID,'look_ahead',sim.simx_opmode_oneshot_wait)
    err_code,l_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Direito", sim.simx_opmode_blocking)
    err_code,r_motor_handle = sim.simxGetObjectHandle(clientID,"Motor_Esquerdo", sim.simx_opmode_blocking)
    
    while not is_near(origem, destino, dist_thresh = 0.25):
        tick = time.time()
        err_code,origem = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
        err_code,origem = sim.simxGetObjectPosition(clientID,origem,-1, sim.simx_opmode_blocking)
        err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
        err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
        print ('posicao robo:', origem)
        print ('orientação do robo:', orientacao_robo)
        print ('caminho:', caminho)
        dist = get_distance(caminho, np.array([0,0])) 
        print ('dist:', dist)
        #loop to determine which point will be the carrot
        for i in range(dist.argmin(), dist.shape[0]):
            if dist[i] < lad and indx <= i:
                indx = i
        #mark the carrot with the sphere
        returnCode=sim.simxSetObjectPosition(clientID,look_ahead_sphere,-1,(caminho[indx,0], caminho[indx,1], 0.005),sim.simx_opmode_oneshot)
        orient_error = np.arctan2(caminho[indx,1], caminho[indx,0])
        #the controllers 
        v_sp = d_controller.control(dist[indx])                     
        om_sp = omega_controller.control(orient_error)
        vr, vl = car_robot_model(v_sp, om_sp)
        err_code,l_motor_handle = sim.simxSetJointTargetVelocity(clientID, l_motor_handle, vr, sim.simx_opmode_oneshot)
        err_code,r_motor_handle = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,vl, sim.simx_opmode_oneshot)
        count += 1
        tock = time.time()                
        dt = tock - tick
        print(dt)
    else:
        print("Chegou ao destino!")
        err_code,l_motor_handle = sim.simxSetJointTargetVelocity(clientID, l_motor_handle, 0, sim.simx_opmode_oneshot)
        err_code,r_motor_handle = sim.simxSetJointTargetVelocity(clientID, r_motor_handle,0, sim.simx_opmode_oneshot)
    return 0
    
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
def controlador_posicao(coordenadas, orientacao):
    kr = kdelta = ksigma = 1
    R = (coordenadas[0]**2+coordenadas[1]**2)**(1/2)
    delta = np.arctan((coordenadas[1]/coordenadas[0]))
    sigma = delta + orientacao
    v = Kr*R*cos(delta)
    w = kdelta*delta + (kr*(delta + ksigma*sigma)*(sen(delta)*cos(delta)))/delta
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

    err_code = sim.simxSetJointTargetVelocity(clientID,l_motor_handle,0.0,sim.simx_opmode_streaming)
    err_code = sim.simxSetJointTargetVelocity(clientID,r_motor_handle,0.0,sim.simx_opmode_streaming)

    err_code,posicao_robo = sim.simxGetObjectHandle(clientID,"Origem", sim.simx_opmode_blocking)
    err_code,posicao_robo = sim.simxGetObjectPosition(clientID,posicao_robo,-1, sim.simx_opmode_blocking)
    err_code,destino_robo = sim.simxGetObjectHandle(clientID,"Destino", sim.simx_opmode_blocking)
    err_code,destino_robo = sim.simxGetObjectPosition(clientID,destino_robo,-1, sim.simx_opmode_blocking)
    err_code,orientacao_robo = sim.simxGetObjectHandle(clientID,"GyroSensor", sim.simx_opmode_blocking)
    err_code,orientacao_robo = sim.simxGetObjectOrientation(clientID,orientacao_robo,-1, sim.simx_opmode_blocking)
    print ('posicao robo:', posicao_robo)
    print ('Destino robo:',destino_robo)
    print ('orientação do robo:', orientacao_robo)


    

    #############################
    #teste controle estabilizante
    caminho, orientation = polinomio(point_init,point_end)
    send_path_4_drawing(caminho, 0.05)

    seguidor_caminho(posicao_robo,destino_robo,caminho)
    print ('Finalizado seguidor de caminho')

    v, w = controlador_posicao(posicao_robo, orientacao_robo)

    #teste controle estabilizante
    #############################

    seguidor_caminho(posicao_robo,destino_robo,orientacao_robo,caminho)
    print ('Finalizado seguidor de caminho')


    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    # while time.time()-startTime < 5:
    #     returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
    #     if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    #         print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
    #     time.sleep(0.005)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
