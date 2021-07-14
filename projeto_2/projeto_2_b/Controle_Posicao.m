clc
clear

format short; % 4 casas decimais em um número flutuante

%Configuração inicial de comunicacao
disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
     %-------------------------------------------Handle-----------------------------------------------------------------------------
     
     [returnCode,left_Motor]=sim.simxGetObjectHandle(clientID,'Motor_Esquerdo',sim.simx_opmode_blocking);
     [returnCode,right_Motor]=sim.simxGetObjectHandle(clientID,'Motor_Direito',sim.simx_opmode_blocking);
     [returnCode,carro]= sim.simxGetObjectHandle(clientID,'Carro',sim.simx_opmode_blocking);
     %------------------------------------------Other Code----------------------------------------------------------------------------
     %Movimentacao
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);
    
     %Posicao e orientacao do robo
     [returnCode,position]=sim.simxGetObjectPosition(clientID,carro,-1,sim.simx_opmode_streaming);
     [returnCode,angulo_robo]=sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_streaming);
      
     %PONTOS FINAIS
     xf = +1.375;
     yf = -0.675;
     
     %Ganhos do controlador
     k_theta = 1.4;
     k_l = 0.05;
     
     %Dados do robô
     rd = 0.0625;
     re = 0.0625;
     B = 0.21;
     
     cont = 0; % contador para testes
     
     while true %tempo de movimentação
     %Get position of Robot
     [returnCode,position]=sim.simxGetObjectPosition(clientID,carro,-1,sim.simx_opmode_buffer); %(Retorna X,Y,Z)
     [returnCode,angulo_robo]=sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_buffer);
     
     %[returnCode,,,angulo_robo_euler,~]=sim.simxGetObjectGroupData(clientID,carro,5,sim.simx_opmode_buffer);
     Theta_Robo = angulo_robo(3) + pi/2;
     
%      if(Theta_Robo < 0)
%          Theta_Robo = Theta_Robo + pi/2;
%      end
          
     %Calculo dos Delta x e y 
     delta_x = xf - position(1);
     delta_y = yf - position(2);
         
     %Calculo do Theta estrela/Referencial
     Theta_ref = atan2(delta_y,delta_x);     
     
     if(Theta_ref < 0)
       Theta_ref = Theta_ref + pi/2;
     end
     
     %Calculo do delta l do referencial e  delta Theta
     delta_l_ref = sqrt((delta_x)^2 + (delta_y)^2);     
     delta_theta = Theta_ref - Theta_Robo;
     
     %Caluclo do delta L
     delta_l = delta_l_ref*cos(delta_theta);%cos(abs(delta_theta));%
     
     %Calculo da velocidade linear e angular
     v = k_l*delta_l;
     w = k_theta * delta_theta;
    
     %velocidades das juntas
     wd = (v/rd) + (B/(2*rd))*w;
     we = (v/re) - (B/(2*re))*w;
         
     sim.simxSetJointTargetVelocity(clientID,left_Motor,we,sim.simx_opmode_blocking);
     sim.simxSetJointTargetVelocity(clientID,right_Motor,wd,sim.simx_opmode_blocking);
                    
     if(delta_l_ref <= 0.05)
        break;
     end
         

     end
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);

     sim.simxFinish(-1);
 end
 
 sim.delete();