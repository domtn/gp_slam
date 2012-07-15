function ShowOccupancyGrids()

OccupancyFileName = 'OccupancyGrid_Kpscaled_finished';
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

% 
imagesc(xlimit, ylimit, M);
colormap(gray);

% figure;
hold on; axis equal;
axis([xlimit(1)-10, xlimit(2)+10, ylimit(1)-10, ylimit(2)+10]);

modelFileName = 'WorstCaseMap';
    Modelpoints = importdata([modelFileName,'.dat'], '\t', 1);
    Model = Modelpoints.data;
% [xlimit, ylimit, Model] = FindMapSize(modelFileName, margin);


% for m=1:size(Model,1)
%     xLineMap = [Model(m,1) Model(m,3)];
%     yLineMap = [Model(m,2) Model(m,4)];
%     plot(xLineMap, yLineMap, '-r', 'LineWidth', 2);
% end
    
end


% function [xlimit, ylimit, Model] = FindMapSize(modelFileName, margin)
% 
%     Modelpoints = importdata([modelFileName,'.dat'], '\t', 1);
%     Model = Modelpoints.data;
%     
% %     if (strcmp(modelFileName, 'map22') == 1)
% %         ModelForChecking = [Model(:,1), Model(:,2);Model(:,3), Model(:,4)];
% %         newPoints = [87.7904, 0.7857; 93.6127, 0.5431;
% %                         163.7229, 0.5431; 170.5155, 0.0579];
% %                     
% %         for i=1:size(newPoints,1)
% %             dist = [];
% %             for j=1:size(ModelForChecking,1)
% %                 dist(j) = Distance(newPoints(i,1),newPoints(i,2), ModelForChecking(j,1), ModelForChecking(j,2));
% %             end                        
% %             [shortestDistance, closestID] = min(dist);
% %             newPoints(i,1) = ModelForChecking(closestID,1);
% %             newPoints(i,2) = ModelForChecking(closestID,2);
% %         end
% %                 
% %         Model = [Model; newPoints(1,1),newPoints(1,2),newPoints(2,1),newPoints(2,2);
% %                         newPoints(3,1),newPoints(3,2),newPoints(4,1),newPoints(4,2)];    
% %     end
%     
% 
%     %% Separate model matrix into 4 vectors
%     x1 = Model(:,1);
%     y1 = Model(:,2);
%     x2 = Model(:,3);
%     y2 = Model(:,4);
%     
%     x1min = (min(x1));
%     x2min = (min(x2));
%     if (x2min < x1min)
%         xmin = x2min; 
%     else
%         xmin = x1min;  
%     end
% 
%     x1max = (max(x1));
%     x2max = (max(x2));
%     if (x2max > x1max)
%         xmax = x2max;
%     else
%         xmax = x1max;
%     end
% 
%     y1min = (min(y1));
%     y2min = (min(y2));
%     if (y2min < y1min)
%         ymin = y2min;
%     else
%         ymin = y1min;
%     end
% 
%     y1max = (max(y1));
%     y2max = (max(y2));
%     if (y2max > y1max)
%         ymax = y2max;
%     else
%         ymax = y1max;
%     end
%     
%     xmin = xmin - margin;
%     xmax = xmax + margin;
%     ymin = ymin - margin;
%     ymax = ymax + margin;
%     xlimit = [xmin xmax];
%     ylimit = [ymin ymax];
%     Model = [Model; xmin, ymin, xmin, ymax;...
%                     xmin, ymax, xmax, ymax;...
%                     xmax, ymax, xmax, ymin;...
%                     xmax, ymin, xmin, ymin];
%     
% end