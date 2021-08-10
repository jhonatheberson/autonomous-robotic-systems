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
    z = 0;
    motion= input('To move the Robot, Press W,A,S,D for Forward, Left, Reverse, Right, and Press x to end\n','s');
    while (motion ~= 'x')
    if (motion=='W' || motion=='w' || motion=='A' || motion=='a' || motion=='S' || motion=='s' || motion=='D' || motion=='d') 
       
%         code here
%         Handle
    [returnCode,left_Motor]=sim.simxGetObjectHandle(clientID,'Motor_Esquerdo',sim.simx_opmode_blocking );
    [returnCode,Right_Motor]=sim.simxGetObjectHandle(clientID,'Motor_Direito',sim.simx_opmode_blocking );
    [returnCode,front_Sensor]=sim.simxGetObjectHandle(clientID,'Sensor_Proximidade',sim.simx_opmode_blocking );
    %[returnCode,camera]=sim.simxGetObjectHandle(clientID,'Vision_sensor',vrep.simx_opmode_blocking );

%     Other Code
   if( motion=='W' ||motion=='w')
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,4,sim.simx_opmode_blocking);    
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,4,sim.simx_opmode_blocking);   
%     pause(1)
%     [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);    
%     [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking); 
   elseif( motion=='A' ||motion=='a')
   [returnCode]=sim.simxSetJointTargetVelocity( clientID,Right_Motor,4,sim.simx_opmode_blocking);
   [returnCode]=sim.simxSetJointTargetVelocity( clientID,left_Motor,0,sim.simx_opmode_blocking);   

%    pause(1)
%    [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking);   
   elseif( motion=='S' ||motion=='s')
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,left_Motor,-4,sim.simx_opmode_blocking);    
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,Right_Motor,-4,sim.simx_opmode_blocking);
%      pause(1)
%     [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking);    
%     [returnCode]=vrep.simxSetJointTargetVelocity( clientID,Right_Motor,0,vrep.simx_opmode_blocking); 
   elseif( motion=='D' ||motion=='d')
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,left_Motor,4,sim.simx_opmode_blocking); 
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,Right_Motor,0,sim.simx_opmode_blocking);

%     [returnCode]=vrep.simxSetJointTargetVelocity( clientID,left_Motor,0,vrep.simx_opmode_blocking); 

   end
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,front_Sensor,sim.simx_opmode_streaming);
    %[returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_streaming);
    [returnCode,leftlinearvelocity, leftangularvelocity]=sim.simxGetObjectVelocity(clientID,left_Motor,sim.simx_opmode_streaming);
    [returnCode,rightlinearvelocity, rightangularvelocity]=sim.simxGetObjectVelocity(clientID,Right_Motor,sim.simx_opmode_streaming);

    [returnCode,leftlinearvelocity,leftangularvelocity]=sim.simxGetObjectVelocity(clientID,left_Motor,sim.simx_opmode_buffer);
    [returnCode,rightlinearvelocity, rightangularvelocity]=sim.simxGetObjectVelocity(clientID,Right_Motor,sim.simx_opmode_buffer);
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,front_Sensor,sim.simx_opmode_buffer );
    %[returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,0,sim.simx_opmode_buffer);
    %imshow(image)
    %fprintf("Obstacle Distance from Robot= %d m \n",norm(detectedPoint));
    %fprintf("Linear Velocity of Left Wheel= %d m/s \n",leftlinearvelocity);
    %fprintf("Linear Velocity of Right Wheel= %d m/s \n",rightlinearvelocity);
    %fprintf("Angular Velocity of Left Wheel= %d rad/s \n",leftangularvelocity);
    %fprintf("Angular Velocity of Right Wheel= %d rad/s \n",rightangularvelocity);

    pause(0.1);
     
  else
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,left_Motor,0,sim.simx_opmode_blocking); 
    [returnCode]=sim.simxSetJointTargetVelocity( clientID,Right_Motor,0,sim.simx_opmode_blocking);   
    
       
    end
    motion= input('To move the Robot, Press W,A,S,D for Forward, Left, Reverse, Right, and Press x to end\n','s');
    %motion = getkeywait(2);
         %% INITIALIZATION

        %Retrieve all handles. Store them in a structure. 

        [res, Velodyne1] = sim.simxGetObjectHandle(clientID, 'velodyneVPL_16', sim.simx_opmode_oneshot_wait);

        res = sim.simxReadVisionSensor(clientID, Velodyne1, sim.simx_opmode_streaming); 
         %% LIDAR Data Streaming
        
        sim.simxCallScriptFunction(clientID, 'velodyneVPL_16', 1, 'getPoints',[],[],[],[],sim.simx_opmode_streaming); % Initialize streaming
        if (motion ~= 'x')
        [returnCode,pointCloudint,pointCloudfloat,pointCloudstr,buffer] = sim.simxCallScriptFunction(clientID, 'velodyneVPL_16', 1, 'getPoints',[],[],[],[],sim.simx_opmode_buffer);
            if (z==0)
                ptcloud = pointCloudfloat;
                z = z+1;
            else
                ptcloud = horzcat(ptcloud,pointCloudfloat);
                z = z+1;
            end
            
        end
    continue
    end
%     vrep.simxFinish(-1); 
    end
sim.simxFinish(-1);
sim.delete();
%% Point Cloud Plotting
try
    for i = 1:length(ptcloud)
        x(i) = ptcloud(3*i+1);
        y(i) = ptcloud(3*i+2);
        z(i) = ptcloud(3*i+3);
    end  
catch 
    scatter3(x,y,z,0.5)
end

L = length(x);
map = binaryOccupancyMap(5,5,100);
map.GridOriginInLocal = [-2.5 -2.5]/1;
%show(map)
setOccupancy(map, [x' y'], ones(L,1))
figure
show(map)