function Generate2DTrainingSetManual()
%% Function name: CommonLaserTestProgram.m
% Lastest changes: 
%%  Functionality: 
%       A common test platform that allows you to verify if a laser sensor
%       simulator is working correctly
%%  How to use it:
%       First Left-click to set the robot's position
%       Second Left-click to set the robot's heading
%       Right-click to exit
%%  This program contains tips on how to read an Occupancy Grid File 
%%  (line 36-57)

clc;
close all;

% Sensor Parameters
angularRes = 90.0; 
alpha1 = 0.0; 
alpha2 = 360.0; 
maxRange = 81.0;  
addNoise = 0;

% Occupancy Grid Map
[occGridFileName, occGridFilePath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Occupancy Grid file');
occGridFile = [occGridFilePath, occGridFileName];

targetFile = [StripExtensionOff(occGridFileName), 'train'];

% Create a name string for the Log File
datetime_string = clock;    
for i=1:size(datetime_string,2)
	targetFile = [targetFile, '_', num2str(ceil(datetime_string(i)))];
end    

targetFile = [targetFile, '.dat'];
	
id = fopen(occGridFile);
for i=1:8
	readin = fgetl(id);
	para(i) = str2num(readin(16:length(readin)));        
end

xMin = para(1); % Bottom Left corner of the map
yMin = para(2);
xMax = para(3); % Top Right corner of the map
yMax = para(4);
imax = para(5); % size of the map (in 'number of voxels')
jmax = para(6);
mapRes = para(7); % map resolution

fclose(id);

occGridParams = [xMin yMin mapRes];
occGridsData = importdata(occGridFile, '\t', 8); 
occGrid = occGridsData.data; 


hold on; 
axis equal;

axis([xMin-1 xMax+1 yMin-1 yMax+1]);
imagesc([xMin xMax], [yMin yMax], occGrid);
colormap(gray);

x = zeros(3,1);

count = 1;

alphaVector = -90.0:90.0:180.0;
alphaVector = alphaVector';


dataSet = [];



disp('Press any key to continue');
pause;


dataSet = [dataSet; 0, 0, 0, 0, (alphaVector')];
while (1)
    
	% Get robot's pose
    [xPose, yPose, buttonPose] = ginput(1);
    if (buttonPose == 3) 
        break;
    end
    
    x(1,1) = xPose;
    x(2,1) = yPose;
    if (count == 1) 
        hPose = plot(x(1), x(2), 'oy');
    else
        set(hPose, 'XData', x(1));
        set(hPose, 'YData', x(2));
    end
    
%     [xHeading, yHeading, buttonHeading] = ginput(1);
%     [thetaHeading, rhoS] = cart2pol(xHeading-xPose, yHeading-yPose);
%     thetaPose = thetaHeading;
	xHeading = xPose;
	yHeading = yPose + 0.2;
	thetaPose = pi/2;
	
    x(3,1) = thetaPose;  

    
    if (count == 1) 
        hHeading = plot ( [xPose xHeading], [yPose, yHeading], '-y');
    else
        set(hHeading, 'XData', [xPose xHeading]);
        set(hHeading, 'YData', [yPose, yHeading]);
	end
    
	[z, correspAlpha] = SonarSimSensor(x, occGrid, occGridParams, alphaVector, maxRange, addNoise); 
    
	dataSet = [dataSet; count, x', z'];
	
    xBeams = [];
    yBeams = [];
    x_impact = [];
    y_impact = [];
    for i=1:length(correspAlpha)
        [x_impact(i), y_impact(i)] = FrameSensorToInertial(xPose, yPose, thetaPose, correspAlpha(i), z(i), 1);
        xBeams = [xBeams, xPose, x_impact(i)];
        yBeams = [yBeams, yPose, y_impact(i)];
    end
    
    if (count == 1) 
        hImpact = plot(x_impact, y_impact, '.r');
        hBeams = plot(xBeams, yBeams, '-r');
    else
        set(hImpact, 'XData', x_impact);
        set(hImpact, 'YData', y_impact);
        set(hBeams, 'XData', xBeams);
        set(hBeams, 'YData', yBeams);
    end
    
    hPose = plot(x(1), x(2), 'oy');
    set(hPose, 'XData', x(1));
    set(hPose, 'YData', x(2));
    
    hHeading = plot ( [xPose xHeading], [yPose, yHeading], '-y');
    set(hHeading, 'XData', [xPose xHeading]);
    set(hHeading, 'YData', [yPose, yHeading]);
        
    
    count = count + 1;
	
	
	
end %(while)

    fid = fopen(targetFile, 'wt');
    fprintf(fid, '%%ID\t');
	fprintf(fid, 'p_x    \t');
	fprintf(fid, 'p_y    \t');
	fprintf(fid, 'p_theta\t');
	
	for i=1:size(alphaVector,1)
		fprintf(fid, ['z_', num2str(alphaVector(i,1))]);
		fprintf(fid, '\t');
	end
	fprintf(fid, '\n');
	fclose(fid);
	
	dlmwrite(targetFile, dataSet, 'delimiter', '\t', '-append', 'newline', 'pc');

    
end



% Last changed: June 16 2008
function [x_InertialFrame, y_InertialFrame] = FrameSensorToInertial(xS, yS, thetaS, param1, param2, NotCart)
% Purpose:  convert polar coordinate of a point in local frame (S) 
%           into the corresponding Cartesan coordinate in global frame (E) 
% inputs:   [xS, yS] = position of the LRF in global frame (E)
%           thetaS = orientation of the LRF (in radian) from the 
%           x-axis of the global frame.
%           param1 = angle of current point "in question" in the local frame
%           (S) in radian
%           param2 = the polar magnitude of the point "in question" in the local
%           frame (S)

    if (NotCart == 1)
        %% Convert local polar coordinate into local Cartesan coordinate
        [x_SensorFrame, y_SensorFrame] = pol2cart(param1, param2);  
    else
        x_SensorFrame = param1;
        y_SensorFrame = param2;
    end    

%% Rotation then translation
    %rotation
    x_rotated = x_SensorFrame*cos(thetaS) - y_SensorFrame*sin(thetaS);
    y_rotated = y_SensorFrame*cos(thetaS) + x_SensorFrame*sin(thetaS);
    
    %translation
    x_InertialFrame = x_rotated + xS;
    y_InertialFrame = y_rotated + yS;
end
