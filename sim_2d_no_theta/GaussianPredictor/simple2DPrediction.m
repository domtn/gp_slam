function simple2DPrediction()


% Particle Grid Poses
[particleGridFileName, particleGridPath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Particle Grid file');
particleGridFileWithoutAngle = StripExtensionOff(particleGridFileName, '_');
particleGridFile = [particleGridPath, particleGridFileWithoutAngle];


% Particle Grid Poses
% [angleConfigFileName, angleConfigPath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Angle Config file');
% angleConfigFile = [angleConfigPath, angleConfigFileName];
angleConfigFile = 'C:\Users\tnguyen\Documents\ESC499\Code\Simple2DSim\GaussianPredictor\oneRectangle_occGrid_particlegrids_2010_1_19_23_44_29\angleConfig.dat';
angles = load(angleConfigFile);


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
		
	
i = 1;

currentParticleGridFile = [particleGridFile, '_', num2str(angles(i,1)), '.dat'];

% Import the grid matrix
grid = importdata(currentParticleGridFile, '\t', 1); 
particleGrid = grid.data;   

gp_hyperparams = [gp_length_scale, gp_sigma_f, gp_sigma_noise];

% Plot the training inputs (poses where groundtruth measurements were
% taken
for training_id = 1:size(TrainingInputs,2)
	xPose = TrainingInputs(1,training_id);
	yPose = TrainingInputs(2,training_id);
	thetaPose = TrainingInputs(3,training_id);	
end

pause;

% Run a loop to perform Gaussian process prediction to find predicted
% measurements at each particle pose (test input)
for particle_id = 1:size(particleGrid,1)

	disp(['particle_id =', num2str(particle_id)]); 

	xPose = particleGrid(particle_id,1);
	yPose = particleGrid(particle_id,2);
	thetaPose = particleGrid(particle_id,3);

	testLocation = [xPose; yPose; thetaPose];

	for observation_id = 1:size(TrainingObservations,2)

		% Perform Gaussian Process prediction
		[predictive_mean, predictive_variance, failcode] = simple1D_GPPredictor(TrainingInputs, ...
																				TrainingObservations(:,observation_id), ... 
																				gp_hyperparams, ...
																				testLocation);
		predictedObservations(observation_id,1) = predictive_mean;

	end

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


