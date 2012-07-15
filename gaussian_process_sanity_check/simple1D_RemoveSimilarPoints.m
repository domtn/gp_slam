% Simple function that find and eliminate repeated input points
% For example, data set that has the first 2 columns like this:
%
% Row  Col_1    Col_2   Col_3   Col_4
% 1      3       5
% 2      3       7
% 3      2       5
% 4      1       10
% 5      3       5
% 6      2       7
% 7      3       7
% Then rows 5 and 7 will be removed, since they are identical to row 1 and
% 2, respectively
% 

function [newData, nRemoved] = simple1D_RemoveSimilarPoints(oldData)
    trainingData = oldData;
    
    first = 1;
    
	nRemoved = 0;
    % While loop that iterates the variable 'first' through all rows
    while(1)
        
        % Trashbin vector to store the indices to be removed
        trashbin = [];
        
        % for-loop to compare the values in entry 'first' with 
        % the rest of the entries in the set
        for second=1:size(trainingData,1)
            if (first==second)
                continue;
            end
            
            if(trainingData(first,1) == trainingData(second,1))
                if(trainingData(first,2) == trainingData(second,2))
                    disp(['Similar input points found at index ', num2str(first), ' and index ', num2str(second)]);
                    trashbin = [trashbin, second];
                else
                    continue;
                end
            end
        end
        
        % If there are entries to be removed
        if ((size(trashbin,2) > 0))            
            nRemoved = nRemoved + size(trashbin,2);
            for i=1:size(trashbin,2)
                trainingData(trashbin(1,i),:) = [];
            end
        end

        
        first = first + 1;
        
        % If end of data set, then quit
        if (first > size(trainingData,1))
            break;
        end
            
    end
    
    newData = trainingData;
end