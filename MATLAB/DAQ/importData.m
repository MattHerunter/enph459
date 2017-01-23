% Function to import directory of .csv files into a single cell array
function data = importData(varargin)
    if(nargin == 1)
        dataDir = varargin{1};
    else
       dataDir = uigetdir(); 
    end
    currentDir = cd(dataDir);
    files = dir;
    
    % Remove any subdirectories from the list of directory contents
    files = files(~[files.isdir]);
    n = numel(files);
    
    % Get the number of columns of data in the .csv
    numSensors = size(csvread(files(1).name,1,0),2);
    
    % Initialize the cell array
    data = cell(n,3+numSensors);
    for ii = 1:numel(files)
        % File names are in format 'fdcXXXrpmXXX_testNumber_timestamp'
        name = files(ii).name;
        
        % Extract FDC, RPM, and test number
        fdc = name(4:regexp(name,'rpm')-1);
        rpm = name((regexp(name,'rpm')+3):(regexp(name,'_.*')-1));
        sep = regexp(name,'_');
        testNum = name((sep(1)+1):(sep(2)-1));
        
        % Read the data from the .csv
        sensors = csvread(name,1,0);
        
        % Store the FDC, RPM, and test number in the first 3 columns
        data(ii,1:3) = {str2double(fdc) str2double(rpm) str2double(testNum)};
        
        % Store the sensor data in the remaining columns
        data(ii,4:(3+numSensors)) = mat2cell(sensors,size(sensors,1),ones(1,numSensors));
    end
    cd(currentDir)
end