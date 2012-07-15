function CreateOccGridMap()
    clc;
    close all;
    
	% original linemap model
    [modelFileName, modelFilePath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Line Map Model file');
    modelFile = [modelFilePath, modelFileName];
    
	% occupancy gridmap model
    targetFileName = [StripExtensionOff(modelFileName), '_occGrid.dat'];


    margin = 1; % unit
    [xlimit, ylimit, Model] = FindMapSize(modelFile, margin);
    
	% Scale axis
	hold on; 
	axis equal; 
	grid on;
	axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);
		
		
	% Plot the lines in the original line maps
	for m=1:size(Model,1)
		xLineMap = [Model(m,1) Model(m,3)];
		yLineMap = [Model(m,2) Model(m,4)];
		plot(xLineMap, yLineMap, '-b', 'LineWidth', 2);
	end
	
	
	laserRange = input('Please enter a value for laserRange		: ');
	laserRes   = 1.0;
	
	
    laserFieldOfView = 360.0;
    nPoints = laserFieldOfView/laserRes;
    laserRes = laserRes/180*pi;
    laserParam = [laserRange, laserRes, laserFieldOfView, nPoints]; 
	
	gridRes = input('Please enter a value for grid resolution: ');
	
    voxelsize = [gridRes gridRes]; % row_height, column width
    imax = round((ylimit(2) - ylimit(1))/voxelsize(1));
    jmax = round((xlimit(2) - xlimit(1))/voxelsize(2));

    M = ones(imax, jmax);
    xlimit = [xlimit(1) xlimit(2)];
    ylimit = [ylimit(1) ylimit(2)];
       
	while (1)
        poseMatrix = [];
        i = 1;
        
        % Plot current occupancy grid map
        imagesc(xlimit, ylimit, M);
        colormap(gray);


        % Scale axis
        hold on; 
        axis equal; 
        grid on;
        axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);
		
		% Plot the lines in the original line maps
        for m=1:size(Model,1)
            xLineMap = [Model(m,1) Model(m,3)];
            yLineMap = [Model(m,2) Model(m,4)];
            plot(xLineMap, yLineMap, '-b', 'LineWidth', 2);
        end
    
        while(1)
            [xPose, yPose, buttonPose] = ginput(1);
            if (buttonPose == 3)
                break;
            end
            poseMatrix(i,1) = xPose;
            poseMatrix(i,2) = yPose;        
            poseMatrix(i,3) = 0;
            circleAngle = 0.0:pi/180:2*pi;
            plot(xPose, yPose, '+r');
            plot(xPose+laserRange*cos(circleAngle), yPose+laserRange*sin(circleAngle), '-r');
            i = i + 1;
        end

        PostMapProcessing(Model, poseMatrix, M, xlimit, ylimit, voxelsize, laserParam, targetFileName);
        
        % Plot the lines in the original line maps
        for m=1:size(Model,1)
            xLineMap = [Model(m,1) Model(m,3)];
            yLineMap = [Model(m,2) Model(m,4)];
            plot(xLineMap, yLineMap, '-b', 'LineWidth', 2);
        end
        
        disp('You have the following options:');
        disp('1. Continue');
        disp('2. Quit');
        your_answer = input('Your selection : ');
        
        if (your_answer == 2)
            break;
        end
        
        close all;        
        id = fopen(targetFileName);
        for i=1:8
            readin = fgetl(id);
            para(i) = str2num(readin(16:length(readin)));        
        end

        xlimit = [para(1)  para(3)];     
        ylimit = [para(2)  para(4)];
        imax = para(5);
        jmax = para(6);
        voxelsize = [para(7), para(8)] ;
        fclose('all');

        % Import the grid matrix
        Mpoints = importdata(targetFileName, '\t', 8); 
        M = Mpoints.data;      
    end

end


function [xlimit, ylimit, Model] = FindMapSize(modelFile, margin)

    Modelpoints = importdata(modelFile, ' ', 1);
    Model = Modelpoints.data;
    
    
    %% Separate model matrix into 4 vectors
    x1 = Model(:,1);
    y1 = Model(:,2);
    x2 = Model(:,3);
    y2 = Model(:,4);
    
    x1min = (min(x1));
    x2min = (min(x2));
    if (x2min < x1min)
        xmin = x2min; 
    else
        xmin = x1min;  
    end

    x1max = (max(x1));
    x2max = (max(x2));
    if (x2max > x1max)
        xmax = x2max;
    else
        xmax = x1max;
    end

    y1min = (min(y1));
    y2min = (min(y2));
    if (y2min < y1min)
        ymin = y2min;
    else
        ymin = y1min;
    end

    y1max = (max(y1));
    y2max = (max(y2));
    if (y2max > y1max)
        ymax = y2max;
    else
        ymax = y1max;
    end
    
    xmin = xmin - margin;
    xmax = xmax + margin;
    ymin = ymin - margin;
    ymax = ymax + margin;
    xlimit = [xmin xmax];
    ylimit = [ymin ymax];
    Model = [Model; xmin, ymin, xmin, ymax;...
                    xmin, ymax, xmax, ymax;...
                    xmax, ymax, xmax, ymin;...
                    xmax, ymin, xmin, ymin];
    
end


