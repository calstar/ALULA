function MatlabCdCalc
%% Initial Reset
% Clear everything
close all; clear;
% clear all
% reset all ports; otherwise might be unable to connect to port
instrreset;

% create our clean up object for interrupt
cleanupObj = onCleanup(@cleanMeUp);

%% User Control
% Linear least square method is used in this code.

% NAME THE TEST FIRST
% The code will read from the previous data, or establish a new file if no
% data present.
% MUST CHANGE NAME OR DELETE PREVIOUS FILE IF DIFFERENT NUMBER OF SENSORS REPORT DATA
fileName = 'OCt082022CdCalcr3';

% NAME THE FOLDER YOU WANT THE TEST TO BE IN
folderName = 'Oct072022CdTest';

% Name the sensors (will be used in data logging and graph titles)
testDevice = 'PT ';

% Name the device that will be next to the flow meter
downStreamPTNum = 3;
upStreamPTNum = 2;

% What is the density of fluid (kg/m^3)
density = 1.27;

% Pulse rate 
% small plastic flow sensor
% PULSE_RATE = 1694.9;

% brass sensor
PULSE_RATE = 1874;

r0 = 0.49;
r1 = 0.2;
L = 2*(r0-r1)/1000;


% How many sensors are you reporting each time? (match with Arduino output)
dataLength = 5;

% How many data points do you want to use to calculate (if use 5,only the last 5 data points will be kept upon stopping the
% program)
dataPointNum = 5;





%% Automated Process Starts here

% n = dataLength;
dataFileExist = 0;
arrayMatch = 1;
prevArray = [];
if exist([fileName,'.xls'])
    dataFileExist = 1;
    prevTable = readtable([fileName,'.xls']);
    prevArray = table2array(prevTable);
    prevArray = prevArray(3:end,:);

    [~,colLength] = size(prevArray);
    if dataLength ~= colLength-1
        ME = MException('MyComponent:newAndOldDataStructureMismatch',['The' ...
            'file you are reading from has a different number of sensor being logged,' ...
            'change the name of the file or delete the original file']);
        arrayMatch = 0;
        throw(ME)
    end

end


% set up dynamic table columns
dataLabels = "Time(ms)";
for n = 2:dataLength-1
    eachLabel = convertCharsToStrings({[testDevice,num2str(n)]});
    dataLabels = [dataLabels,eachLabel];
end

eachLabel = "volume flow (m^3/s)";
dataLabels = [dataLabels,eachLabel];

eachLabel = "break Interval";
dataLabels = [dataLabels,eachLabel];

eachLabel = "mass flow (kg/s)";
dataLabels = [dataLabels,eachLabel];

eachLabel = "Cd Value";
dataLabels = [dataLabels,eachLabel];

eachLabel = "Reynolds Num";
dataLabels = [dataLabels,eachLabel];
finalArray = [];





% set up serial object
%serialPortName = '/dev/cu.SLAB_USBtoUART'
serialPortName = 'COM14'; % on Windows would be COMx
%s = serialport(serialPortName,115200);
s = serial(serialPortName,'BaudRate',115200);

% for storing data sequentially in rawData
rawData = [];
i = 1;

% open serial port and read data
serialPortOpened = 1;
try
    fopen(s);
    flushinput(s);
    % fscanf(s);
catch
    serialPortOpened = 0;
    availablePorts = convertStringsToChars(serialportlist("all"));
    [~,numOfPorts] = size(availablePorts);
    portNames = [];
    for n = 1:numOfPorts
        if n == 1
            portNames = availablePorts{n};
        else
            portNames = [portNames, ', ',availablePorts{n}];
        end
    end

    ME = MException('MyComponent:fopenFailed',['Failed to fopen the serial ' ...
        'port, check the serial port name. Available ports: ' portNames]);
    throw(ME)

end

if serialPortOpened == 1
%    str = fscanf(s);

       while(1)
            str = split(fscanf(s));

           if length(str) ~= dataLength+1
               continue;
           end
           if ~isnumeric(str2double(str{1}))
               continue;
           end
           % each data line represents one sensor data
         

%            rawData(i,1) = str2double(str{1})-rawData(1,1)

           for n = 1:dataLength
               rawData(i,n) = str2double(str{n})
           end

           % time interval from last one
           if i ~= 1
               rawData(i,n+1) = rawData(i,1) - rawData(i-1,1)
           end

           i = i+1;
       end


end
    function cleanMeUp()
%         rawData
        if arrayMatch == 1 && serialPortOpened == 1
%            instrreset;

            % saves data to file (or could save to workspace)
            fprintf('saving test data as %s.xls\n',fileName)

            if ~exist(folderName, 'dir')
                mkdir(folderName);
                addpath(folderName);
                fprintf("test data folder created\n");
            else
                fprintf("folder already exists\n")
                addpath(folderName);
            end

            rawData = rawData(2:end,:);

            finalArray = finalizeData(rawData,downStreamPTNum,upStreamPTNum);
%             dataLabels

            testDataTable = array2table(finalArray,'VariableNames',dataLabels);

            fileString = fileName + ".xls";

            if dataFileExist == 1
                writetable(testDataTable,fileString,'WriteMode','overwrite')
                movefile(fileString,folderName);
            else
                writetable(testDataTable,fileName,"FileType","spreadsheet");
                movefile(fileString,folderName);

            end

        end
        fclose(s);
        instrreset;
    end

    function returnArray = finalizeData(arrayInput, leftPTNum, rightPTNum)

        rawData
        volFlow = rawData(:,5)
        volFlowMetric = volFlow*0.003785;
        massFlowRate = volFlowMetric*density;

        leftPTVals = arrayInput(:,leftPTNum);
        rightPTVals = arrayInput(:,rightPTNum);
        cdVals = massFlowRate./(sqrt(2*density*(rightPTVals-leftPTVals)));

        area = 17e-6;
        cdVals = cdVals/area;

        arrayInput(:,5) = volFlowMetric
        reynolds = density*L*(volFlowMetric/pi*(L/2*0.69))/0.001;
%         arrayOut = arrayInput(:,5)/PULSE_RATE;
        returnArray = [arrayInput,massFlowRate,cdVals,reynolds];
        fprintf("Mass Flow Total: %8f\n",sum(massFlowRate.*arrayInput(:,1)/1000))

       % 1 gal/min = 6.309e-5 m^3/s

    end










end