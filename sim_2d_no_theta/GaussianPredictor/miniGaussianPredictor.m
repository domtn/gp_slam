function miniGaussianPredictor()

close all;

% Occupancy Grid Map
[occGridFileName, occGridFilePath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Occupancy Grid file');
occGridFile = [occGridFilePath, occGridFileName];
% occGridFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSim\TrainingDataGeneration\oneRectangle_occGrid.dat';

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
% trainingDataFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSim\DataGeneration\oneRectangle_occGridtrain_2010_1_22_13_5_45.dat';
trainingData = load(trainingDataFile);


TrainingInputs = trainingData(2:size(trainingData,1), ...
	             2:4);
			 
% Training input			 
TrainingInputs = TrainingInputs';

% Training observations
TrainingObservations = trainingData(2:size(trainingData,1), ...
								   5:size(trainingData,2));
observationAngles = trainingData(1, 5:size(trainingData,2))';
observationAngles = observationAngles.*pi/180.0;


predictedObservations = zeros(size(TrainingObservations,2), 1);  
predictedVariance = zeros(size(TrainingObservations,2), 1);  



% Obtain parameters for Gaussian Process Prediction
gp_sigma_noise          = input('Please enter a value for gp_sigma_noise        : ');
max_gp_length_scale     = 2.5; % input('Please enter a value for max_gp_length_scale   : ');
min_gp_length_scale     = 1.15; % input('Please enter a value for min_gp_length_scale   : ');    
step_gp_length_scale    = 0.1; % input('Please enter a value for step_gp_length_scale  : ');
max_gp_sigma_f          = 2.5; % input('Please enter a value for max_gp_sigma_f        : ');
min_gp_sigma_f          = 1.15; % input('Please enter a value for min_gp_sigma_f        : ');
step_gp_sigma_f         = 0.1; % input('Please enter a value for step_gp_sigma_f       : ')

gp_length_scale = 1;      
gp_sigma_f = 1;


xBeams = [];
yBeams = [];
x_impact = [];
y_impact = [];	
	


figure;
hold on; 
axis equal;

axis([xMin-2 xMax+2 yMin-2 yMax+2]);
imagesc([xMin xMax], [yMin yMax], occGrid);
colormap(gray);

gp_hyperparams = [gp_length_scale, gp_sigma_f, gp_sigma_noise];

% Plot the training inputs (poses where groundtruth measurements were
% taken
for training_id = 1:size(TrainingInputs,2)

	xPose = TrainingInputs(1,training_id);
	yPose = TrainingInputs(2,training_id);
	thetaPose = TrainingInputs(3,training_id);

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


while(1)
	
	% Get robot's pose
	[xPose, yPose, buttonPose] = ginput(1);
	if (buttonPose == 3) 
		break;
	end

	if (count == 1) 
		hPose = plot(xPose, yPose, 'oy');
	else
		set(hPose, 'XData', xPose);
		set(hPose, 'YData', yPose);
	end

% 	[xHeading, yHeading, buttonHeading] = ginput(1);
% 	[thetaHeading, rhoS] = cart2pol(xHeading-xPose, yHeading-yPose);

	xHeading = xPose;
	yHeading = yPose + 0.5;
	thetaHeading = pi/2;
	
	thetaPose = thetaHeading;

	if (count == 1) 
		hHeading = plot ( [xPose xHeading], [yPose, yHeading], '-y');
	else
		set(hHeading, 'XData', [xPose xHeading]);
		set(hHeading, 'YData', [yPose, yHeading]);
	end
	
	drawnow;
	testLocation = [xPose; yPose; thetaPose];

	
	xBeams = [];
	yBeams = [];
    x_impact = [];
    y_impact = [];	
	
	for observation_id = 1:size(TrainingObservations,2)
		% Perform Gaussian Process prediction
		[predictive_mean, predictive_variance, failcode] = simple1D_GPPredictor(TrainingInputs, ...
																				TrainingObservations(:,observation_id), ... 
																				gp_hyperparams, ...
																				testLocation);

		predictedObservations(observation_id,1) = predictive_mean;
		predictedVariance(observation_id,1) = predictive_variance;
		display(['Observation ', num2str(observation_id), ' complete']);
		
		
		
		[x_impact(observation_id), y_impact(observation_id)] = FrameSensorToInertial(xPose, ...
																		   yPose, ...
																		   thetaPose, ...
																		   observationAngles(observation_id,1),... 
																		   predictedObservations(observation_id,1), 1);

		xBeams = [xBeams, xPose, x_impact(observation_id)];
		yBeams = [yBeams, yPose, y_impact(observation_id)];
		

		
		
		
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
		
			
	drawnow;

% 	pause;   
    
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


