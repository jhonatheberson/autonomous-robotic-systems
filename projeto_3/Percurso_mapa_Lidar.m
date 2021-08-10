clc
clear

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
        
        
      load ('mapa scan lidar.mat')
      show(map)

      robotRadius = 0.4;
      mapInflated = copy(map);
      inflate(mapInflated,robotRadius);
      show(mapInflated)

      %Construct PRM and Set Parameters
      %Create a mobileRobotPRM object and define the associated attributes.
      prm = mobileRobotPRM;
      %Assign the inflated map to the PRM object.
      prm.Map = mapInflated;
      %Define the number of PRM nodes to be used during PRM construction.
      prm.NumNodes = 400;
      %Define the maximum allowed distance between two connected nodes on the map.
      prm.ConnectionDistance = 1;

      %Define start and end locations on the map for the path planner to use.
      startLocation = [-1.75 -1.75];
      endLocation = [1.5 -1.75];

      %Search for a path between start and end locations using the findpath function
      path = findpath(prm, startLocation, endLocation);

      %writematrix(path,'Caminho probabilistico.csv')

      %Display the PRM solution.
      show(prm)

        robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
        controller = controllerPurePursuit;
        controller.Waypoints = path;
        controller.DesiredLinearVelocity = 0.1;
        controller.MaxAngularVelocity = 1.0;
        controller.LookaheadDistance = 1.0;
        robotInitialLocation = path(1,:);
        robotGoal = path(end,:);
        initialOrientation = 90;
        robotCurrentPose = [robotInitialLocation initialOrientation]';
        distanceToGoal = norm(robotInitialLocation - robotGoal);
        goalRadius = 0.1;
         rd = 0.0625;
         re = 0.0625;
         %B = 0.13;
         B = 0.10;
         sampleTime = 0.1;
         % Determine vehicle frame size to most closely represent vehicle with plotTransforms
        frameSize = robot.TrackWidth/2.5;
        vizRate = rateControl(1/sampleTime);
        release(controller);
      
     
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
    show(map);
    hold all

    %Plot path each instance so that it stays persistent while robot mesh
    %moves
    plot(path(:,1), path(:,2),"k--d")
    
    %Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, 'MeshFilePath', 'groundvehicle.stl', 'Parent', gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([-2.5 2.5])
    ylim([-2.5 2.5])
    
    waitfor(vizRate);
                    
 
     end
     

     [returnCode]=sim.simxSetJointTargetVelocity(clientID,left_Motor,0,sim.simx_opmode_blocking);
     [returnCode]=sim.simxSetJointTargetVelocity(clientID,right_Motor,0,sim.simx_opmode_blocking);

     sim.simxFinish(-1);
    end
    
%Gerar o path para exportar para o V-rep
x=[];
y=[];
z=[];
 for i=1:length(path)
     x(i) = path(i,1);
     y(i) = path(i,2);
     z(i) = 0.005;
 end
 MAT = [x' y' z'];
 %writematrix(MAT,'Caminho Mapa Lidar.csv')
