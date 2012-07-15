% Function to wrap any angle value to a corresponding value [-pi/2, pi/2]
function outputAngle = angleWrap(inputAngle)
	
% 	angleUnit = pi;
	angleUnit = 180;
	
	
	epsilon = 10^(-9);
	if (abs(inputAngle-0) < epsilon)
		outputAngle = 0;
		return;
	end
		
	signness = (inputAngle/abs(inputAngle));
	
	
	
	outputAngle = signness*mod(abs(inputAngle), 2*angleUnit);
	
	
	if (outputAngle < -angleUnit)
		outputAngle = 2*angleUnit + outputAngle;
	elseif (outputAngle > angleUnit)
		outputAngle = (-1)*(2*angleUnit - outputAngle);
% 	else
% 		outputAngle = 2*angleUnit + outputAngle;
	end
end