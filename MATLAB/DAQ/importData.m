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
    
    names = {files.name}';
    C = regexp(names,'fdc(\d+).*_(\d+)_','tokens');
    C = [C{:}]';
    A = zeros(numel(C),1);
    for jj=1:numel(C)
        c=C{jj};
        A(jj) = str2double(c{1})+str2double(c{2});
    end
    [~,order]=sort(A);
    files=files(order);
    % Get the number of columns of data in the .csv
    numSensors = size(csvread(files(1).name,1,0),2);
    
    % Initialize the cell array
    data = cell(n,4+numSensors);
    for ii = 1:numel(files)
        % File names are in format 'fdcXXXrpmXXXtsXXX_testNumber_timestamp'
        name = files(ii).name;
        
        settings = regexp(name,'fdc(\d+)rpm(\d+)ts(\d+)_(\d+)','tokens');
        settings = settings{:};
%         % Extract FDC, RPM, and test number
%         fdc = name(4:regexp(name,'rpm')-1);
%         rpm = name((regexp(name,'rpm')+3):(regexp(name,'_.*')-1));
%         sep = regexp(name,'_');
%         testNum = name((sep(1)+1):(sep(2)-1));
%         ts = regexp(name,'ts(\d+)_');
        
        % Read the data from the .csv
        sensors = csvread(name,1,0);
        
        % Store the FDC, RPM, sample time, and test number in the first 4 columns
        data(ii,1:4) = {str2double(settings{1}) str2double(settings{2}) str2double(settings{3}) str2double(settings{4})};
        
        % Store the sensor data in the remaining columns
        data(ii,5:(4+numSensors)) = mat2cell(sensors,size(sensors,1),ones(1,numSensors));
    end
    cd(currentDir)
end