clear all
clc
dh=.4;
X=-5:dh:5;
Y=-5:dh:5;
[X,Y]=meshgrid(X,Y);
x0=[0;4];
xT=[0;-4];
Kr=50;
Vr=Kr./sqrt((X-x0(1)).^2+(Y-x0(2)).^2);
[fxr,fyr]=gradient(Vr,dh,dh);
Ka=40;
Va=.5.*Ka.*((X-xT(1)).^2+(Y-xT(2)).^2);
[fxa,fya]=gradient(Va,dh,dh);
%obstaculos
cx1=-1;
cy1=0;
cx2=1;
cy2=0;
cx3=2;
cy3=1;
cx4=3;
cy4=2;
rad=.4;
th=(0:.1*dh:2).*pi;
xo=rad.*cos(th);
yo=rad.*sin(th);
xo1=xo-cx1;
yo1=yo-cy1;
xo2=xo-cx2;
yo2=yo-cy2;

Vo1=[];
Vo2=[];

Vo=zeros(size(Vr));
Ko=5;

for i=1:length(th)
 Vo1=Ko./sqrt((X-xo1(i)).^2+(Y-yo1(i)).^2);
 Vo2=Ko./sqrt((X-xo2(i)).^2+(Y-yo2(i)).^2);

 Vo=Vo+Vo1+Vo2;

end
[fxo,fyo]=gradient(Vo,dh,dh);
fX=-fxr-fxa-fxo-fyo;
fY=-fyr-fya-fyo+fxo;
quiver(X,Y,fX,fY,'k')
title('Potential Field of Area')
xlabel('X-Axis')
ylabel('Y-Axis')
hold on
plot(x0(1),x0(2),'ro',xT(1),xT(2),'ro')
fill(xo1,yo1,'b',xo2,yo2,'b')
plot(xo1,yo1,'b',xo2,yo2,'b')
hold off
ss=1;
k=1;
xp=[];
yp=[];
zp=[];
xp(1)=x0(1);
yp(1)=x0(2);
zp(1) = 0.0;
ix=[];
iy=[];
jx=[];
jy=[];
fxx=0;
fyy=0;
while ss

 Pw=sqrt(((X-xp(k)).^2)+((Y-yp(k)).^2));
 xw(k)=min(min(Pw));

 [iix,iiy]=find(Pw==xw(k));
 ix(k)=iix(1);
 iy(k)=iiy(1);

 fx1=fX(ix(k),iy(k));
 fy1=fY(ix(k),iy(k));

 fxx(k)=fx1./norm(fX);
 fyy(k)=fy1./norm(fY);

 ff(k,:)=[fX(ix(k),iy(k)),fY(ix(k),iy(k))];

 xp(k+1)=xp(k)+dh*(fxx(k));
 yp(k+1)=yp(k)+dh*(fyy(k));
 zp(k+1) = 0.0;

 if (sqrt((xp(k+1)-xT(1)).^2+(yp(k+1)-xT(2)).^2)<=0.4)
 ss=0;
 end

 k=k+1;
end
hold
plot([xp],[yp],'r')
MAT = [xp' yp' zp'];
%writematrix(MAT,'Curva Potencial.csv')
figure
surf(X,Y,(Vr)+(Va)+(Vo))
title('Superfície Potencial')

%Configuração inicial de comunicacao
disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    %send=send_path('send_path_4_drawing'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
    
%     function [returnCode] = send_path_4_drawing(path, sleep_time)
%     %the bigger the sleep time the more accurate the points are placed but you have to be very patient :D
%         for i=path
%             %packedData=sim.simxPackFloats(i.flatten());
%             raw_bytes = (ctypes.c_ubyte * len(sim.simxPackFloats(i.flatten())))
%             returnCode=sim.simxWriteStringStream(clientID, "path_coord", raw_bytes, sim.simx_opmode_oneshot)
%             time.sleep(sleep_time)
% 
%         end
%     end

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

path = [xp' yp'];     
robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 2.0;
controller.LookaheadDistance = 2.0;
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = -90;
robotCurrentPose = [robotInitialLocation initialOrientation]';
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.5;
 rd = 0.06;
 re = 0.06;
 B = 0.13;
 sampleTime = 0.1;
 % Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/2.5;
vizRate = rateControl(1/sampleTime);
release(controller);


      
%      for i=path
%             %packedData=sim.simxPackFloats(i.flatten());
%             raw_bytes = (uint8 * len(sim.simxPackFloats(i.flatten())))
%             [returnCode]=sim.simxWriteStringStream(clientID, "path_coord", raw_bytes, sim.simx_opmode_oneshot)
%             time.sleep(0,05)
% 
%     end
     %send_path_4_drawing(path, 0.05)
    
     
     while( distanceToGoal > goalRadius ) %tempo de movimentação
         
         
           release(controller);

         [v, omega] = controller(robotCurrentPose);
            
         % Get the robot's velocity using controller inputs
         
         
         %Velocidade das juntas
         wd = (v/rd) + (B/(2*rd))*omega;
         we = (v/re) - (B/(2*re))*omega;
         
         sim.simxSetJointTargetVelocity(clientID,left_Motor,we,sim.simx_opmode_blocking);
         sim.simxSetJointTargetVelocity(clientID,right_Motor,wd,sim.simx_opmode_blocking);
         
         %Parametro para pegar a posicao e orientacao
         [returnCode,position]=sim.simxGetObjectPosition(clientID,carro,-1,sim.simx_opmode_buffer); %(Retorna X,Y,Z)
         [returnCode,angulo_robo]=sim.simxGetObjectOrientation(clientID,carro,-1,sim.simx_opmode_buffer);
         
         vel = derivative(robot, robotCurrentPose, [v omega]);
         x = position(1);
         y = position(2);
         theta = angulo_robo(3);
         Pose = [x y theta]';
          % Update the current pose
          robotCurrentPose = Pose;
        %robotCurrentPose = robotCurrentPose + vel*sampleTime; 

        % Re-compute the distance to the goal
        distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
        %distanceToGoal = norm(position(1:2) - robotGoal(:));
         
          % Update the plot
    hold off
    %show(map);
    hold all

    %Plot path each instance so that it stays persistent while robot mesh
    %moves
    plot(path(:,1), path(:,2),"k--d")
    
    %Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    
    xlim([-5 5])
    ylim([-5 5])
    waitfor(vizRate);
                    
 
     end
     

     [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);

     sim.simxFinish(-1);
 end
 
 
 sim.delete();