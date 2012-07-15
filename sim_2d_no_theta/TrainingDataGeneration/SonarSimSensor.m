function [z, correspAlpha, alpha_z] = SonarSimSensor(x, ...
											occGrid, ...
											occGridParams, ...
											alphaVector, ...
											maxRange, ...
											addNoise) 
%% Function name: laserOccGridSimulator.m
%  Lastest changes: 
%%  Funtionality: the laser simulator that works with an occupancy grid map
%   
%%  Inputs:
%       x - Pose of form [xs, ys, thetas(rad)]
%       occGrid = Grid matrix (1 is in, 0 is out)
%       occGridParams = [xMin, yMin, mapRes] in metres;
%                           [xMin, yMin] = coordinate of lower left corner of the OccGrid Map
%                           mapRes = the resoltion of the OccGrid, in metres
%                                    this value will be used as the range resolution of the
%                                    laser sensor
%       angularRes : angular resolution in degrees
%       alpha1: min angle in degrees
%       alpha2: max angle in degrees
%       maxRange: maximum range of the laser, in metres 
%       addNoise: set to 1 to addNoise, to 0 not to add Noise
%
%%  Outputs:
%       z = ranges, in metres [column]
%       correspAlpha = corresponding alpha values of the ranges in z(between
%                      alpha1 to alpha2) [column]
%%

xMin = occGridParams(1);
yMin = occGridParams(2);
mapRes = occGridParams(3);

correspAlpha = alphaVector.*(pi/180.0);

% convert 'angular resolution', 'alpha1', and 'alpha2' to radian
% resRad = angularRes/180*pi;
% minAngle = alpha1/180*pi;
% maxAngle = alpha2/180*pi;

% compute the number of impact points
% nPoints = ceil((maxAngle-minAngle)/resRad) + 1;
nPoints = size(correspAlpha,1);

% set 'range resolution' to the 'map resolution '
rangeRes = mapRes;

% Rename the components of the pose vector
xS = x(1);  
yS = x(2);
thetaS = x(3);


% Pre-compute the sins and cosines  
cosAngle = zeros(nPoints,1);
sinAngle = zeros(nPoints,1);

for i=1:nPoints
    alpha = correspAlpha(i,1);
    cosAngle(i,1) = cos( alpha + thetaS);
    sinAngle(i,1) = sin( alpha + thetaS);    
end


z = zeros(nPoints,1);
% correspAlpha = zeros(nPoints,1);
% 
% for i=1:nPoints
% 	correspAlpha(i,1) = (alpha1 + angularRes*(i-1))/180*pi;
% end

% Calculate the dimension of the map (number of grids in rows, and number
% of grids in column)
imax = size(occGrid,1);
jmax = size(occGrid,2);

% Calculate the grid coordinate of the sensor's position in the occupancy
% grid
i_pose= ceil(abs((yS-yMin))/mapRes);
j_pose = ceil(abs((xS-xMin))/mapRes);


if ( (i_pose < 1) || (i_pose > imax )...
    || (j_pose < 1) || (j_pose > jmax )  )
    % The sensor is outside the map's area -> no ray-tracing
	
elseif (occGrid(i_pose, j_pose) == 1)
    % The sensor is inside wall regions -> no ray-tracing  
	
% If all clear, then ray-trace
else  
    % For-loop over all the beams
    for i=1:nPoints
        
        %initialize beam length (initial range) to zero
        laserbeam_length = 0.0; 
        
        % Ray-tracing loop
        while(laserbeam_length < maxRange)     
            
            % Calculate the coordinate of the hat of the beam
            x_beamHat = xS + laserbeam_length*cosAngle(i);
            y_beamHat = yS + laserbeam_length*sinAngle(i);        

            
            % Convert the hat of the beam to grid coodinate 
            i_temp = ceil((y_beamHat-yMin)/mapRes);
            j_temp = ceil((x_beamHat-xMin)/mapRes);
            
            %Check if the hat of the beam is in or outside map area
            if ( (i_temp < 1) || (i_temp > imax)...
                || (j_temp < 1) || (j_temp > jmax)  )
                    z(i) = laserbeam_length;
                    break;
            end
            
            %If hit an 'out' cell (grid value of 0), take the current length
            %as the range
            if (occGrid(i_temp, j_temp) == 1)
                    z(i) = laserbeam_length;
                    break;      
            end   
            
            %increase beam_length by rangeRes
            laserbeam_length = laserbeam_length + rangeRes;
        end
        
        % If beam_length reaches maxRange, then take maxRange as range
        if (abs(laserbeam_length - maxRange) <= mapRes)
            z(i) = maxRange;
        end
        
        % Calculate the values of the cooresponding alpha
        % correspAlpha(i,1) = (alpha1 + angularRes*(i-1))/180*pi;         
    end

end


if (addNoise==1)
    % Add some noise to the readings
%     noise = zeros(nPoints,1);
    noise = randn(nPoints,1);
    z = z + rangeRes*noise*maxRange/32.0;
    % Account for range resolution
    z = rangeRes * ceil( z / rangeRes );
end


end




