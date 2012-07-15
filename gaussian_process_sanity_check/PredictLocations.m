% Simple Gaussian Process Regression for Sanity Check 
% Sensor Readings are Inputs, Robot Locations are Targets
function PredictLocations()

close all;

figure; 
axis equal;
hold on;

H = 10;  % Height of the map
W = 15;  % Width of the map

% plot the map
xMap = [0 W W 0 0];
yMap = [0 0 H H 0];
plot(xMap, yMap, 'k-');
xlabel('x');
ylabel('y');

% Generate Poses on the map
x = W.*rand(1,100);
y = H.*rand(1,100);
Poses = [x; y; ones(1,size(x,2))];

% Remove any idential data points
Poses = simple1D_RemoveSimilarPoints(Poses');
Poses = Poses';

plot(Poses(1,:), Poses(2,:), 'b.');

% Generate Ranges using from the location of the robot
% and the dimensions of the map
Ranges = simSensor(Poses, H, W);

% Obtain parameters for Gaussian Process Prediction
gpSigmaNoise     = 0; % input('Please enter a value for gp_sigma_noise        : ');
gpLengthScale    = 3; % input('Please enter a value for max_gp_length_scale   : ');
gpSigmaF         = 1; % input('Please enter a value for step_gp_sigma_f       : ');
gpHyperparams = [gpLengthScale, gpSigmaF, gpSigmaNoise];

testOuputMean1 = -1*ones(1, size(Poses,2));
testOuputMean2 = -1*ones(1, size(Poses,2));
testOuputMean3 = -1*ones(1, size(Poses,2));
testOuputMean4 = -1*ones(1, size(Poses,2));

%Run loop to perform Guassian process Prediction at all data points in
%training set
for i=1:size(Poses,2)
	
	% Pick test input out of all training Data
	testInput = Ranges(:,i);
	designMatrix = Ranges;
	designMatrix(:,i) = [];
	
	% Create target vectors
	targets1 = x';
	targets2 = y';
	targets1(i,:) = [];
	targets2(i,:) = [];	
	
	
	groundtruth1 = x(1,i);
	groundtruth2 = y(1,i);	
	

	% Perform Gaussian Process Prediction to predict x
	[testOuputMean1(1,i), testOutputVariance1(1,i), failCode] = simple1D_GPPredictor(designMatrix, ...
																			       targets1, ...
																				   gpHyperparams, ...
																				   testInput);						   
	% Perform Gaussian Process Prediction to predict y							   
	[testOuputMean2(1,i), testOutputVariance2(1,i), failCode] = simple1D_GPPredictor(designMatrix, ...
																			       targets2, ...
																				   gpHyperparams, ...
																				   testInput);

	% plot
	if (i==1)
		hplot1 = plot(groundtruth1, groundtruth2, 'ro');
		hplot2 = plot(testOuputMean1(1,i), testOuputMean2(1,i), 'r+');
		legend('map', 'robot locations', 'groundtruth', 'prediction');
	else
		set(hplot1, 'XData', groundtruth1, ...
                                   'YData', groundtruth2);
		set(hplot2, 'XData', testOuputMean1(1,i), ...
                                   'YData', testOuputMean2(1,i));							   
	end					  
	
	display('Please Press Any key to move on to the next point');
	pause;
	
end



end

% Simple sim sensor that generates range readings from map with no
% obstacles
function z = simSensor(pose, mapHeight, mapWidth)

W = mapWidth;
H = mapHeight;

T = [ -1  0  W;
	   0 -1  H;
	   1  0  0;
	   0  1  0];

z = T*pose;
	

end