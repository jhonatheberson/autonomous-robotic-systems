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
 
    point_init = [0.0000,0.0000, 0.0000];
    point_end = [1.5250, 1.8000, 0.0000];
    teta_i = 0;
    teta_f = 0;
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
      
     %Polinomio interpolador
     [X, Y, a, b, Mat] = faz_polonomio(position,point_end, teta_i, teta_f, 0.01);
     
     %Ganhos do controlador
     k_theta = 0.5;
     k_l = 0.03;
     
     i = 1;
     %Velocidade definida como constante
     v = 0.4;
     rd = 0.06;
     re = 0.06;
     B = 0.13;
     
     while true %tempo de movimentação
         
         %Parametro para pegar a posicao e orientacao
         [returnCode,position]=sim.simxGetObjectPosition(clientID,carro,-1,sim.simx_opmode_buffer); %(Retorna X,Y,Z)
         [returnCode,angulo_robo]=sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_buffer);
                          
         %calculo da menor distância do robô a curva
         [ponto_curva, Delta_l, lambda] = distance2curve(Mat(:,1:2),[position(1),position(2)],'linear');
            
                  
         Theta_robo = angulo_robo(3) + pi/2;      
         %disp(Theta_robo);

         %raio de giro
         dx = a(2) +2*a(3)*lambda + 3*a(4)*(lambda^2);
         dy = b(2) + 2*b(3)*lambda + 3*b(4)*(lambda^2);
         d2x = 2*a(3) + 6*a(4)*lambda;
         d2y = 2*b(3) + 6*b(4)*lambda;
         r = ((((dx^2)+(dy^2))^1.5)/((d2y*dx)-(d2x*dy)));              
         k = (1/r);
         
         %delta theta
         theta_SF = atan((b(2) + 2*b(3)*(lambda) + 3*b(4)*(lambda^2))/(a(2) + 2*a(3)*(lambda) + 3*a(4)*(lambda^2)));%*180/pi;
         Delta_theta = Theta_robo - theta_SF;
         
         %Garantir o sinal correto de Delta L 
         theta_posicao_robo = atan2(position(2),position(1)); %angulo de posicao do robo
         theta_ref = atan2(ponto_curva(2),ponto_curva(1)); %angulo da curva que está mais próxima ao robo
         
         if(theta_ref>theta_posicao_robo) %Sinal do delta L
             Delta_l = -Delta_l;
         end
         
         ErroL(i) = Delta_l;
         i = i+1;
         
         %Controle
         u = -(k_theta*Delta_theta + (k_l*Delta_l*v*sin(Delta_theta)/Delta_theta));
         
         %Velocidade Angular
         w = u + ((k*v*cos(Delta_theta))/(1-(k*Delta_l)));
         
         %Velocidade das juntas
         wd = (v/rd) + (B/(2*rd))*w;
         we = (v/re) - (B/(2*re))*w;
         
         sim.simxSetJointTargetVelocity(clientID,left_Motor,we,sim.simx_opmode_blocking);
         sim.simxSetJointTargetVelocity(clientID,right_Motor,wd,sim.simx_opmode_blocking);
                    
         if(lambda==1)
             break;
         end
         
     end
     
      %Plot do erro Delta l
      figure(2)
      plot(ErroL)
      grid on;
      ylabel('Delta L')
      xlabel('Interação')
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);

     sim.simxFinish(-1);
 end
 
 sim.delete();