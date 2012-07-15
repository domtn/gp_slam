function PostMapProcessing(Model, poseMatrix, M, xlimit, ylimit, voxelsize, laserParam, targetFileName)
         
    for i=1:size(poseMatrix,1)
        pose = poseMatrix(i, :);
        [M, x_beamHat, y_beamHat] = ProcessPose(Model, pose, M, xlimit, ylimit, voxelsize, laserParam);
    end      
    
    fid = fopen(targetFileName, 'wt');
    fprintf(fid, 'xmin          = %5.3f \n', xlimit(1));
    fprintf(fid, 'ymin          = %5.3f \n', ylimit(1));
    fprintf(fid, 'xmax          = %5.3f \n', xlimit(2));
    fprintf(fid, 'ymax          = %5.3f \n', ylimit(2));
    fprintf(fid, 'row           = %f \n', size(M, 1));
    fprintf(fid, 'col           = %f \n', size(M, 2));
    fprintf(fid, 'row voxelsize = %f \n', voxelsize(1));
    fprintf(fid, 'col voxelsize = %f \n', voxelsize(2));

    fclose(fid);

    dlmwrite(targetFileName, M, 'delimiter', '\t', '-append', 'newline', 'pc');
    
    
    imagesc(xlimit, ylimit, M);
    colormap(gray);
    
    hold on; 
    axis equal; 
    grid on;
    axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);
    
    
   
end


function [M, x_beamHat, y_beamHat]  = ProcessPose(Model, pose, M, xlimit, ylimit, voxelsize, laserParam)

    laserPose = pose;
    laserPose = roundToPrecision(laserPose, 4);
    laserRange = laserParam(1);
    laserRes = laserParam(2);
    laserFieldOfView = laserParam(3);
    nPoints = laserParam(4);
  
    CloseByLines = FindCloseByLines(Model, laserPose, laserRange);
    
    imax = size(M,1);
    jmax = size(M,2);
   
    [i_pose, j_pose] = whichVoxel(laserPose, voxelsize, xlimit(1), ylimit(1));
    M(i_pose, j_pose) = 0;   % change from 1
    
    
    % Precompute the trigs
    cosAngle = 1:nPoints;
    sinAngle = 1:nPoints;
    
    alpha = 0;    
    for i=1:nPoints
        alpha = alpha + laserRes;
        Angle = laserPose(3) + alpha;
        cosAngle(i) = cos(Angle);
        sinAngle(i) = sin(Angle);
    end
   
    optionalTheta = 0;
    
    rangeres = voxelsize(1);
    
    alpha = 0;
    
    x_beamHat = [];
    y_beamHat = [];
    for i = 1:nPoints
        laserbeam_length = 0;

        while(laserbeam_length < laserRange)
            laserbeam_length = laserbeam_length + rangeres;
            x_beamHat(i) = laserPose(1) + laserbeam_length*cosAngle(i);
            y_beamHat(i) = laserPose(2) + laserbeam_length*sinAngle(i);
            nLinesIntersected = 0;
            for j=1:size(CloseByLines,1)
                validIntesectionFound = CheckForIntersection(CloseByLines(j,1),...
                                                            CloseByLines(j,2),...
                                                            CloseByLines(j,3),...
                                                            CloseByLines(j,4),...
                                                            laserPose(1),laserPose(2),x_beamHat(i), y_beamHat(i));
                if (validIntesectionFound == 1)
                    nLinesIntersected = nLinesIntersected + 1;
                    break;
                end
			end
            if (nLinesIntersected == 0)
                [i_temp, j_temp] = whichVoxel([x_beamHat(i), y_beamHat(i), optionalTheta], voxelsize, xlimit(1), ylimit(1));
                M(i_temp, j_temp) = 0;
            else
                break;
            end            
        end
        alpha = alpha + laserRes;    
    end
    
        
end


function [i, j] = whichVoxel(pose, voxelsize, xmin, ymin)
    i = ceil(abs((pose(2)-ymin))/voxelsize(1));%row
    j = ceil(abs((pose(1)-xmin))/voxelsize(2));%column
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
    ExtractedModel = [ExtractedModel; xTopLeftBox, yTopLeftBox, xTopRightBox,yTopRightBox; ...
                                  xTopRightBox, yTopRightBox, xBottomRightBox,yBottomRightBox;... 
                                  xBottomRightBox, yBottomRightBox, xBottomLeftBox,yBottomLeftBox; ...
                                  xBottomLeftBox, yBottomLeftBox, xTopLeftBox,yTopLeftBox];
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
