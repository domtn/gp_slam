function GPLeaveOneOut()

close all;

% Occupancy Grid Map
[occGridFileName, occGridFilePath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Occupancy Grid file');
occGridFile = [occGridFilePath, occGridFileName];
% occGridFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSimNoThetaDependentOutputs\MapGenerator\emptyMap_occGrid.dat';

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
% trainingDataFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSimNoThetaDependentOutputs\TrainingDataGeneration\emptyMap_occGridtrain_2010_2_18_23_18_12.dat';
trainingData = load(trainingDataFile);


TrainingPoses = trainingData(2:size(trainingData,1), ...
	             2:4);
			 
% Training input			 
TrainingPoses = TrainingPoses';

% Training observations
TrainingObservations = trainingData(2:size(trainingData,1), ...
								    5:size(trainingData,2));
observationAngles = trainingData(1, 5:size(trainingData,2))';
observationAngles = observationAngles.*pi/180.0;


predictedObservations = zeros(size(TrainingObservations,2), 1);  
predictedVariance = zeros(size(TrainingObservations,2), 1);  


xBeams = [];
yBeams = [];		
x_impact = [];
y_impact = [];		

figure;
hold on; 
axis equal;

axis([xMin-1 xMax+1 yMin-1 yMax+1]);
imagesc([xMin xMax], [yMin yMax], occGrid);
colormap(gray);

hp_v = [1.0, 1.0, 1.0, 1.0];
hp_w = [1.0, 1.0, 1.0, 1.0];
hp_A = [0.1, 0.1, 0.1, 0.1];
hp_B = [0.1, 0.1, 0.1, 0.1];
hp_sigma = [0.0, 0.0, 0.0, 0.0];
hp_mu = [0.2, 0.2, 1.57, 1.0];
	
	
gp_hyperparams = [hp_v; hp_w; hp_A; hp_B; hp_sigma; hp_mu];

% Plot the training inputs (poses where groundtruth measurements were
% taken
for training_id = 1:size(TrainingPoses,2)

	xPose = TrainingPoses(1,training_id);
	yPose = TrainingPoses(2,training_id);
	thetaPose = TrainingPoses(3,training_id);

	[x_heading, y_heading] = FrameSensorToInertial(xPose, ...
												   yPose, ...
												   thetaPose, 0.0, 2.0, 1);        
	plot([xPose, x_heading], [yPose, y_heading], 'b-');        
	plot(xPose, yPose, 'bo');

end

pause;
	
	% Run a loop to perform Gaussian process prediction to find predicted
	% measurements at each particle pose (test input)
	
count = 1;

for i=1:size(TrainingPoses,2)	

	xPose = TrainingPoses(1,i);
	yPose = TrainingPoses(2,i);
	thetaPose = TrainingPoses(3,i);	
	testInput = [xPose; yPose; thetaPose];
	
	
	designMatrix = TrainingPoses;
	designMatrix(:,i) = [];
	
	targetMatrix = TrainingObservations;
	targetMatrix(i,:) = [];
	
	xBeams = [];
	yBeams = [];
	

	[predictive_mean, predictive_variance, failcode] = dependentGPPredict(designMatrix, ...
																		  targetMatrix, ... 
															              gp_hyperparams, ...
																		  testInput);
	predictedObservations = predictive_mean;
		
	for observation_id = 1:size(TrainingObservations,2)		
		[x_impact(1,observation_id), y_impact(1,observation_id)] = FrameSensorToInertial(xPose, ...
																					     yPose, ...
																					     thetaPose, ...
																					     observationAngles(observation_id,1),... 
																					     predictedObservations(observation_id,1), 1);

		xBeams = [xBeams, xPose, x_impact(1,observation_id)];
		yBeams = [yBeams, yPose, y_impact(1,observation_id)];	

	end
	
	if (count == 1) 						
		hBeams = plot(xBeams, yBeams, '-r');
		hImpact = plot(x_impact, y_impact, '.r');
	else
		set(hImpact, 'XData', x_impact);
		set(hImpact, 'YData', y_impact);
		set(hBeams, 'XData', xBeams); 
		set(hBeams, 'YData', yBeams);
	end

	drawnow;
		
	pause;   
    
    hold off;
     
	count = count + 1;
end


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


