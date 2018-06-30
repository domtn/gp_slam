function plotTrainingData()

close all;
clc;

% Occupancy Grid Map
[occGridFileName, occGridFilePath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Occupancy Grid file');
occGridFile = [occGridFilePath, occGridFileName];
% occGridFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSim\DataGeneration\oneRectangle_occGrid.dat';

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


% Training Data Poses
[trainingDataFileName, trainingDataPath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Training Data file');
trainingDataFile = [trainingDataPath, trainingDataFileName];
% trainingDataFile = 'C:\Users\tnguyen\Documents\Thesis\Code\Simple2DSim\DataGeneration\oneRectangle_occGridtrain_2010_1_22_13_5_45.dat';

trainingData = load(trainingDataFile);

TrainingInputs = trainingData(2:size(trainingData,1), ...
                              2:4);
TrainingObservations = trainingData(2:size(trainingData,1), ...
                                    5:size(trainingData,2));
observationAngles = trainingData(1, 5:size(trainingData,2))';

observationAngles = observationAngles.*pi/180.0;

xBeams = [];
yBeams = [];
x_impact = zeros(1,size(observationAngles,1));
y_impact = zeros(1,size(observationAngles,1));

i = 1;
figure;
hold on; 
axis equal;

axis([xMin-20 xMax+20 yMin-20 yMax+20]);
imagesc([xMin xMax], [yMin yMax], occGrid);
colormap(gray);


for training_id = 1:size(TrainingInputs,1)

    disp([training_id, '=', num2str(training_id)]);

    xPose = TrainingInputs(training_id,1);
    yPose = TrainingInputs(training_id,2);
    thetaPose = TrainingInputs(training_id,3);

    observation = TrainingObservations(training_id, :)';

    xBeams = [];
    yBeams = [];

    for beam_id=1:size(observationAngles,1)
        [x_impact(1,beam_id), y_impact(1,beam_id)] = FrameSensorToInertial(xPose, ...
                                                                            yPose, ...
                                                                            thetaPose, ...
                                                                            observationAngles(beam_id,1),... 
                                                                            observation(beam_id,1), 1);
        xBeams = [xBeams, xPose, x_impact(1,beam_id)];
        yBeams = [yBeams, yPose, y_impact(1,beam_id)];
    end
    size(observation)

    if (training_id == 1) 
        hImpact = plot(x_impact, y_impact, '.r');
        hBeams = plot(xBeams, yBeams, '-r');
    else
        set(hImpact, 'XData', x_impact);
        set(hImpact, 'YData', y_impact);
        set(hBeams, 'XData', xBeams);
        set(hBeams, 'YData', yBeams);
    end

    [x_heading, y_heading] = FrameSensorToInertial(xPose, ...
                                                    yPose, ...
                                                    thetaPose, 0.0, 2.0, 1);        
    plot([xPose, x_heading], [yPose, y_heading], 'y-');        
    plot(xPose, yPose, 'yo');

%   pause;

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
    y_rotated = x_SensorFrame*sin(thetaS) + y_SensorFrame*cos(thetaS);

    x_InertialFrame = x_rotated + xS;
    y_InertialFrame = y_rotated + yS;   
end

