function ModifyOccGridMap()
    
    OccupancyFileName = 'OccupancyGrid_Kemi_05';
    targetFileName = OccupancyFileName;
    id = fopen([OccupancyFileName, '.dat']);
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
    Mpoints = importdata([OccupancyFileName, '.dat'], '\t', 8); 
    M = Mpoints.data;   
    
    % Plot current occupancy grid map
    imagesc(xlimit, ylimit, M);
    colormap(gray);
    
    % Scale axis
    hold on; 
    axis equal; 
    grid on;
    axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);
    
    
    laserRange = 32.0;
    laserRes = 1.0;
    laserFieldOfView = 360.0;
    nPoints = laserFieldOfView/laserRes;
    laserRes = laserRes/180*pi;
    laserParam = [laserRange, laserRes, laserFieldOfView, nPoints]; 
    
    modelFileName = 'map22';
    margin = 1;
    [xlimit, ylimit, Model] = FindMapSize(modelFileName, margin);
    
    voxelsize = [0.5 0.5]; % row_height, column width
    imax = round((ylimit(2) - ylimit(1))/voxelsize(1));
    jmax = round((xlimit(2) - xlimit(1))/voxelsize(2));

%     M = zeros(imax, jmax);
    xlimit = [xlimit(1) xlimit(2)];
    ylimit = [ylimit(1) ylimit(2)];
       
    imagesc(xlimit, ylimit, M);
    colormap(gray);
    
    hold on; axis equal; grid on;
    axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);
    for m=1:size(Model,1)
        xLineMap = [Model(m,1) Model(m,3)];
        yLineMap = [Model(m,2) Model(m,4)];
        plot(xLineMap, yLineMap, '-b', 'LineWidth', 2);
    end
%     plot(xS,yS,'y-');
    poseMatrix = [];
    i = 1;
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
        plot(xPose+laserRange*cos(circleAngle), yPose+laserRange*sin(circleAngle), '-w');
        i = i + 1;
    end
    
    PostMapProcessing(Model, poseMatrix, M, xlimit, ylimit, voxelsize, laserParam, targetFileName);

end

function [x_SensorFrame, y_SensorFrame] = FrameInertialToSensor(x_InertialFrame, y_InertialFrame, xS, yS, thetaS)

x_translated = x_InertialFrame - xS;
y_translated = y_InertialFrame - yS; 

x_SensorFrame = x_translated*cos(thetaS) + y_translated*sin(thetaS);
y_SensorFrame = y_translated*cos(thetaS) - x_translated*sin(thetaS);

end

function [x_InertialFrame, y_InertialFrame] = FrameSensorToInertial(xS, yS, thetaS, param1, param2, NotCart)
    if (NotCart == 1)
        %% Convert local polar coordinate into local Cartesan coordinate
        [x_SensorFrame, y_SensorFrame] = pol2cart(param1, param2);  
    else
        x_SensorFrame = param1;
        y_SensorFrame = param2;
    end

    x_rotated = x_SensorFrame*cos(thetaS) - y_SensorFrame*sin(thetaS);
    y_rotated = y_SensorFrame*cos(thetaS) + x_SensorFrame*sin(thetaS);

    x_InertialFrame = x_rotated + xS;
    y_InertialFrame = y_rotated + yS;   
end

function [i, j] = whichVoxel(pose, voxelsize, xmin, ymin)
    i = ceil(abs((pose(2)-ymin))/voxelsize(1));%row
    j = ceil(abs((pose(1)-xmin))/voxelsize(2));%column
end

function [xlimit, ylimit, Model] = FindMapSize(modelFileName, margin)

    Modelpoints = importdata([modelFileName,'.dat'], '\t', 1);
    Model = Modelpoints.data;
    if (strcmp(modelFileName, 'map22') == 1)
        ModelForChecking = [Model(:,1), Model(:,2);Model(:,3), Model(:,4)];
        newPoints = [87.7904, 0.7857; 93.6127, 0.5431;
                        163.7229, 0.5431; 170.5155, 0.0579];
                    
        for i=1:size(newPoints,1)
            dist = [];
            for j=1:size(ModelForChecking,1)
                dist(j) = Distance(newPoints(i,1),newPoints(i,2), ModelForChecking(j,1), ModelForChecking(j,2));
            end                        
            [shortestDistance, closestID] = min(dist);
            newPoints(i,1) = ModelForChecking(closestID,1);
            newPoints(i,2) = ModelForChecking(closestID,2);
        end
                
        Model = [Model; newPoints(1,1),newPoints(1,2),newPoints(2,1),newPoints(2,2);
                        newPoints(3,1),newPoints(3,2),newPoints(4,1),newPoints(4,2)];    
    end
    

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

function CloseByLines = FindCloseByLines(Model, sensorPose, range)
    
    xTopLeftBox = roundToPrecision((sensorPose(1) - range), 4);
    yTopLeftBox = roundToPrecision((sensorPose(2) + range), 4);
    xTopRightBox = roundToPrecision((sensorPose(1) + range), 4);
    yTopRightBox = roundToPrecision((sensorPose(2) + range), 4);
    xBottomLeftBox = roundToPrecision((sensorPose(1) - range), 4);
    yBottomLeftBox = roundToPrecision((sensorPose(2) - range), 4);
    xBottomRightBox = roundToPrecision((sensorPose(1) + range), 4);
    yBottomRightBox = roundToPrecision((sensorPose(2) - range), 4);
    %%% Extracting Line Models that are inside the box or intersecting the
    %%% sides of the box
    ExtractedModel = [];

    for i=1:size(Model,1)
        ChosenLine = 0;
        %Check if both of the endpoints are inside the Box

        %Take each endpoint and create a line to connect it with a corner of 
        %the box, and see if this line intersect any of the sides of the box         
        TotalLinesToCornerIntersected = ...
                               CheckForIntersection(Model(i,1),Model(i,2),... % POINT1-2 to TOP LEFT
                                    xTopLeftBox, yTopLeftBox,   xBottomLeftBox,yBottomLeftBox,xBottomRightBox,yBottomRightBox)+...
                               CheckForIntersection(Model(i,1),Model(i,2),...
                                    xTopLeftBox, yTopLeftBox,   xTopRightBox,yTopRightBox,xBottomRightBox,yBottomRightBox)+...
                                    ...
                               CheckForIntersection(Model(i,1),Model(i,2),... % POINT1-2 to TOP RIGHT
                                    xTopRightBox, yTopRightBox,   xBottomRightBox,yBottomRightBox,xBottomLeftBox,yBottomLeftBox)+...
                               CheckForIntersection(Model(i,1),Model(i,2),...
                                    xTopRightBox, yTopRightBox,   xBottomLeftBox,yBottomLeftBox,xTopLeftBox,yTopLeftBox)+...
                                    ...
                               CheckForIntersection(Model(i,1),Model(i,2),... % POINT1-2 to BOTTOM LEFT
                                    xBottomLeftBox, yBottomLeftBox,   xTopLeftBox,yTopLeftBox,xTopRightBox,yTopRightBox)+...
                               CheckForIntersection(Model(i,1),Model(i,2),...
                                    xBottomLeftBox, yBottomLeftBox,   xTopRightBox,yTopRightBox,xBottomRightBox,yBottomRightBox)+...
                                    ...
                               CheckForIntersection(Model(i,1),Model(i,2),... % POINT1-2 to BOTTOM RIGHT
                                    xBottomRightBox, yBottomRightBox,   xTopRightBox,yTopRightBox,xTopLeftBox,yTopLeftBox)+...
                               CheckForIntersection(Model(i,1),Model(i,2),...
                                    xBottomRightBox, yBottomRightBox,   xBottomLeftBox,yBottomLeftBox,xTopLeftBox,yTopLeftBox)+...
                                    ...
                                    ...
                               CheckForIntersection(Model(i,3),Model(i,4),... % POINT1-2 to TOP LEFT
                                    xTopLeftBox, yTopLeftBox,   xBottomLeftBox,yBottomLeftBox,xBottomRightBox,yBottomRightBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    xTopLeftBox, yTopLeftBox,   xTopRightBox,yTopRightBox,xBottomRightBox,yBottomRightBox)+...
                                    ...
                               CheckForIntersection(Model(i,3),Model(i,4),... % POINT1-2 to TOP RIGHT
                                    xTopRightBox, yTopRightBox,   xBottomRightBox,yBottomRightBox,xBottomLeftBox,yBottomLeftBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    xTopRightBox, yTopRightBox,   xBottomLeftBox,yBottomLeftBox,xTopLeftBox,yTopLeftBox)+...
                                    ...
                               CheckForIntersection(Model(i,3),Model(i,4),... % POINT1-2 to BOTTOM LEFT
                                    xBottomLeftBox, yBottomLeftBox,   xTopLeftBox,yTopLeftBox,xTopRightBox,yTopRightBox)+...
                               CheckForIntersection(Model(i,1),Model(i,4),...
                                    xBottomLeftBox, yBottomLeftBox,   xTopRightBox,yTopRightBox,xBottomRightBox,yBottomRightBox)+...
                                    ...
                               CheckForIntersection(Model(i,3),Model(i,4),... % POINT1-2 to BOTTOM RIGHT
                                    xBottomRightBox, yBottomRightBox,   xTopRightBox,yTopRightBox,xTopLeftBox,yTopLeftBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    xBottomRightBox, yBottomRightBox,   xBottomLeftBox,yBottomLeftBox,xTopLeftBox,yTopLeftBox);
        if (TotalLinesToCornerIntersected == 0)
            ChosenLine = ChosenLine + 1;
        end

        %Check if this specific line in the model intersects with any of the sides of the box   
        TotalModelLinesIntersected = CheckForIntersection(Model(i,3),Model(i,4),...
                                    Model(i,1),Model(i,2),   xTopLeftBox,yTopLeftBox,xTopRightBox,yTopRightBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    Model(i,1),Model(i,2),   xTopRightBox,yTopRightBox,xBottomRightBox,yBottomRightBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    Model(i,1),Model(i,2),   xBottomRightBox,yBottomRightBox,xBottomLeftBox,yBottomLeftBox)+...
                               CheckForIntersection(Model(i,3),Model(i,4),...
                                    Model(i,1),Model(i,2),   xBottomLeftBox,yBottomLeftBox,xTopLeftBox,yTopLeftBox);

        if (TotalModelLinesIntersected > 0)
            ChosenLine = ChosenLine + 1;
        end

        if (ChosenLine > 0)
            ExtractedModel = [ExtractedModel; Model(i,1), Model(i,2), Model(i,3), Model(i,4)]; 
        end
    end
%     ExtractedModel = [ExtractedModel; xTopLeftBox, yTopLeftBox, xTopRightBox,yTopRightBox; ...
%                                   xTopRightBox, yTopRightBox, xBottomRightBox,yBottomRightBox;... 
%                                   xBottomRightBox, yBottomRightBox, xBottomLeftBox,yBottomLeftBox; ...
%                                   xBottomLeftBox, yBottomLeftBox, xTopLeftBox,yTopLeftBox];
    CloseByLines = ExtractedModel;

end

%%% Helper functions:
% CheckLineLimits
% LineIntersection
% roundToPrecision
% return infinity 
% function [intersectExistsFlag, coordinatesOfIntersect] = CheckForIntersection(line1, line2)
% end
 
function validIntesectionFound = CheckForIntersection(x1,y1,x2,y2,x4,y4,x5,y5)
    [xIntersect, yIntersect] = LineIntersection(x1,y1, ...
                                            x2,y2,x4,y4,x5,y5);
                                        
    if ( (CheckLineLimits(xIntersect, yIntersect, x1,y1,x2,y2) == 1) && ...
            (CheckLineLimits(xIntersect, yIntersect, x4,y4,x5,y5) == 1) )
        validIntesectionFound = 1;
    else
        validIntesectionFound = 0;
    end
end

function btwLimits = CheckLineLimits(x_inter, y_inter, x1, y1, x2, y2)
%% function name: CheckLineLimits.m
%  Check if a point (x_inter, y_inter) is between 2 other points (x1, y1)
%  and (x2, y2)
%  Returns 1 if yes and 0 if no
%%
    btwLimits = 0;
    if( ( (x_inter <= x1)&& (x_inter >= x2) ) || ((x_inter >= x1) && (x_inter <= x2)))
        if ( ( (y_inter <= y1)&& (y_inter >= y2) ) || ((y_inter >= y1) && (y_inter <= y2)))
            btwLimits = 1;
        end
    end
end

function [x, y] = LineIntersection(x1, y1, x2, y2, x3, y3, x4, y4)
    det = (x1-x2)*(y3-y4) - (y1-y2)*(x3 - x4);
    if (det == 0)
        x = Inf;
        y = Inf;
    else
        x = ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4 - y3*x4))/det;
        y = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4 - y3*x4))/det;
        
        % Round up or down to 4 decimal places if necessary 
        x = roundToPrecision(x, 4);
        y = roundToPrecision(y, 4);
    end
end


function x_rounded  = roundToPrecision(x, decimalplaces)
    temp = x.*(10^decimalplaces);
    if ( abs(temp- floor(temp)) < 0.5)
        x_rounded = floor(temp)./(10^decimalplaces);
    else
        x_rounded = ceil(temp)./(10^decimalplaces);
    end
    
end

function d = Distance(x1, y1, x2, y2)
    d = sqrt((x1 - x2)^2 + (y1 - y2)^2);
end


