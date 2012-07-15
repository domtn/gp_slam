% Simple function to strip the file extension off a file name
% StripExtensionOff('inputStr.dat') = 'inputStr'
function nameStr = StripExtensionOff(inputStr)
    nameStr = [];
   
%     extStrId = 0;
    % Go through input string and search for the dot
    for i=1:size(inputStr,2)
        if (strcmp(inputStr(1,i),'.') == 0)
            nameStr = [nameStr, inputStr(1,i)];
        else
            extStrId = i;
            break;
        end 
    end
    
%     extStr = inputStr(extStrId:size(inputStr,2));
    
end