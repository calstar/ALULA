function SensorCalibrator
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
fileName = 'PTCalibration_Oct29_newnew';

% NAME THE FOLDER YOU WANT THE TEST TO BE IN
folderName = 'Oct29PTNum_Newnew';

% Name the sensors (will be used in data logging and graph titles)
testDevice = 'PT ';

% How many sensors are you reporting each time? (match with Arduino output)
dataLength = 7;

% How many data points do you want to use to calculate (if use 5,only the last 5 data points will be kept upon stopping the
% program)
dataPointNum = 5;



%% Automated Process Starts here

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
dataLabels = [];
for n = 1:dataLength
    eachLabel = convertCharsToStrings({[testDevice,num2str(n)]});
    dataLabels = [dataLabels,eachLabel];

end
eachLabel = convertCharsToStrings({[testDevice,'Readings']});
dataLabels = [dataLabels,eachLabel];


finalArray = [];
reading = [];


% set up serial object
%serialPortName = '/dev/cu.SLAB_USBtoUART'
 serialPortName = 'COM14'; % on Windows would be COM#
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
    while(1)
        str = split(fscanf(s));
        length(str)

        if length(str) ~= dataLength+1
            continue;
            t = "uayayaydaishdiushfniusdjfbsiufbdsiufdfbis"
        end


        if ~isnumeric(str2num(str{1}))
            m = str2num(str{1});
            continue;
        else

            m = str2num(str{1});
            % each data line represents one sensor data
            for n = 1:dataLength
                rawData(i,n) = str2double(str{n});
            end

            if i > dataPointNum
                rawData(i-dataPointNum,:) = [];
            end

            i = i+1;
        end
    end
end
    function cleanMeUp()
        if arrayMatch == 1 && serialPortOpened == 1
%             instrreset;

            % saves data to file (or could save to workspace)
            fprintf('saving test data as %s.xls\n',fileName)

            prompt = "What is the pressure gage reading? (Numbers only) \n";
            reading = input(prompt);

            %         str2double(reading);
            %         while (isnumeric(reading) == false)
            %             prompt = "What is the pressure gage reading?"
            %             reading = input(prompt);
            %         end
            %         reading = str2double(reading);
            if ~exist(folderName, 'dir')
                mkdir(folderName);
                addpath(folderName);
                fprintf("test data folder created\n");
            else
                fprintf("folder already exists\n")
                addpath(folderName);
            end


            % calculate mean values

            for n = 1:dataLength
                meanArray(1,n) = mean(rmoutliers(rawData(:,n)));
            end
            processArray = [meanArray,reading];
            processArray = [prevArray;processArray];

            a = [];
            b = [];
            endsol = [];
            for j = 1:length(dataLabels)-1
                X = processArray(:,j);
                Y = processArray(:,end);
                coefficients = polyfit(X,Y,1)';
                a = [a;coefficients(1)]
                b = [b;coefficients(2)]
                endsol = [endsol,coefficients];


            end


            finalArray = [[a',NaN];[b',NaN];processArray];

            testDataTable = array2table(finalArray,'VariableNames',dataLabels);

            fileString = fileName + ".xls";

            if dataFileExist == 1
                writetable(testDataTable,fileString,'WriteMode','overwrite')
                movefile(fileString,folderName);
            else
                writetable(testDataTable,fileName,"FileType","spreadsheet");
                movefile(fileString,folderName);

            end
            dataProcessingGraphing(processArray,endsol)

        end
        fclose(s);
        instrreset;
    end

    function dataProcessingGraphing(array,solution)
        figure;
        set(gcf, 'PaperSize', [10 10]);
        plotNumber = length(array(1,:))-1;

        for k = 1:plotNumber
            nexttile
            sortedArray = sort(array(:,k));

            x = linspace(sortedArray(1),sortedArray(end),100);
            titleString = [testDevice,num2str(k)];
            plot(array(:,k),array(:,end),'o')
            title(titleString);
            hold on
            y = solution(1,k)*x+solution(2,k);

            plot(x,y)
            hold off

        end












    end



end


