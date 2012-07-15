function AngleInterpolation()

clc;

% Load training data into memory
% Training Data Poses
[trainingDataFileName, trainingDataPath] = uigetfile({'*.dat';'*.txt';'*.*'}, 'Please pick a Training Data file');
trainingDataFile = [trainingDataPath, trainingDataFileName];
% trainingDataFile = 'C:\Users\tnguyen\Documents\Thesis\Code\Simple2DSim\DataGeneration\oneRectangle_occGridtrain_2010_1_22_13_5_45.dat';
trainingData = load(trainingDataFile);

TrainingInputs = trainingData(2:size(trainingData,1), ...
	             2:4);
TrainingObservations = trainingData(2:size(trainingData,1), ...
								   5:size(trainingData,2));
observationAnglesDeg = trainingData(1, 5:size(trainingData,2));

newObservationAngles = zeros(1, size(observationAnglesDeg,2));


targetFile = [StripExtensionOff(trainingDataFileName), 'interpolated'];
% Create a name string for the Log File
datetime_string = clock;    
for i=1:size(datetime_string,2)
	targetFile = [targetFile, '_', num2str(ceil(datetime_string(i)))];
end    

targetFile = [trainingDataPath, targetFile, '.dat'];



interpolatedTrainingData = [];

interpolatedTrainingData = [0, 0, 0, 0, observationAnglesDeg];
count = 1;


for training_id = 1: size(TrainingInputs,1)
	
	xPose = TrainingInputs(training_id,1);
	yPose = TrainingInputs(training_id,2);
	thetaPose = TrainingInputs(training_id,3);
	
	observation = TrainingObservations(training_id,:);
	
	interpolatedTrainingData = [interpolatedTrainingData; count, xPose, yPose, thetaPose, observation];
	
	
	newObservation = zeros(1,size(observation,2));
	
% 	observation
	
	angle_id = 3;
	
	
	for angle_id = 1:size(observationAnglesDeg,2)
		
		count = count + 1;
		
		newThetaPose = angleWrap(thetaPose*180.0/pi + observationAnglesDeg(1,angle_id) + 180)*pi/180;
				
% 		thetaPose
% 		disp(['observationAngleRotated = ', num2str(observationAnglesDeg(1,angle_id))]);
% 		newThetaPose			
% 		disp('observationAngles = ');
% 		observationAnglesDeg
% 		
% 		for observation_id = 1:size(observationAnglesDeg,2)
% 			newObservationAnglesDeg(1,observation_id) = observationAnglesDeg(1,observation_id)-observationAnglesDeg(1,angle_id);
% 			newObservationAnglesDeg(1,observation_id) = angleWrap(newObservationAnglesDeg(1,observation_id));
% 
% 
% 			
% 		end
% 		
% 		disp(['observationAngleRotated = ', num2str(observationAnglesDeg(1,angle_id))]);
% 		disp('newObservationAngles = ');
% 		newObservationAnglesDeg	
% 		[sortedObservationAnglesDeg, sortedIndex] = sort(newObservationAnglesDeg,2, 'ascend');			
% 		disp('sortedObservationAngles = ');
% 		sortedObservationAnglesDeg
		
		
		nAngles = size(observationAnglesDeg,2);
		
		for j=1:nAngles			
			new_id = (angle_id+j);			
			if ( new_id > nAngles)
				new_id = new_id - nAngles;
			end			
			newObservation(1,j) = observation(1,new_id);
		end	

		interpolatedTrainingData = [interpolatedTrainingData; count, xPose, yPose, newThetaPose, newObservation];
	end	
	
% 	return;
	
	count = count + 1;
	
end

fid = fopen(targetFile, 'wt');
fprintf(fid, '%%ID\t');
fprintf(fid, 'p_x    \t');
fprintf(fid, 'p_y    \t');
fprintf(fid, 'p_theta\t');

for i=1:size(observationAnglesDeg,2)
	fprintf(fid, ['z_', num2str(observationAnglesDeg(1,i), 2)]);
	fprintf(fid, '\t');
end

fprintf(fid, '\n');
fclose(fid);

dlmwrite(targetFile, interpolatedTrainingData, 'delimiter', '\t', '-append', 'newline', 'pc');


	
end

