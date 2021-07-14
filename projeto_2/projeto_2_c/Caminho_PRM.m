image = imread('mapa.png');
grayimage = rgb2gray(image);

bwimage = grayimage < 50;

map = binaryOccupancyMap(bwimage,100);
map.GridOriginInLocal = [-2.5 -2.5]/1;
show(map)



%O robo tem 40cm de comprimento, simulando que ele seja um círculo com raio
%de 20cm.

robotRadius = 0.2;
mapInflated = copy(map);
inflate(mapInflated,robotRadius);
show(mapInflated)

%Construct PRM and Set Parameters
%Create a mobileRobotPRM object and define the associated attributes.
prm = mobileRobotPRM;
%Assign the inflated map to the PRM object.
prm.Map = mapInflated;
%Define the number of PRM nodes to be used during PRM construction.
prm.NumNodes = 200;
%Define the maximum allowed distance between two connected nodes on the map.
prm.ConnectionDistance = 1;

%Find a Feasible Path on the Constructed PRM

%Define start and end locations on the map for the path planner to use.
startLocation = [-1.75 -1.5];
endLocation = [2.0 2.0];

%Search for a path between start and end locations using the findpath function
path = findpath(prm, startLocation, endLocation);

%writematrix(path,'Caminho probabilistico.csv')

%Display the PRM solution.
show(prm)