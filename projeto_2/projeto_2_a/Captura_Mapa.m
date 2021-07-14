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
        
[returnCode,Vision_sensor]=sim.simxGetObjectHandle(clientID,'Mapa',sim.simx_opmode_blocking);

sim.simxGetVisionSensorImage2(clientID,Vision_sensor,0,sim.simx_opmode_streaming);
while (clientID~=-1)
    [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,Vision_sensor,0,sim.simx_opmode_buffer);
    if (returnCode==sim.simx_return_ok)
        imshow(image)
    end
end


    end