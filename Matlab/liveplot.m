function liveplot
% Clear everything
close all; clear all;
% reset all ports; otherwise might be unable to connect to port
% instrreset;

% create our clean up object for interrupt
cleanupObj = onCleanup(@cleanMeUp);


% Arduino Output
dataLength = 14;

% NAME THE TEST FIRST (only change the second part)
fileName = [datestr(now,'yyyy-mm-dd_HHMMSS'),'_test3'];

% set up data monitoring frequency
pauseTime = 0.05;
% set up plotting frequency
plotTime = 0.2;
% % check variable size
% SerialPrintSize = 14;

% frequency
% fetchFrequency = 1/pauseTime;

% observation time window
observationInterval = 5;


% time conversion factor
% timeFactor = 24 * 60 * 60;

data = [];

calibrationData = [1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0;1,0];
% [a, b]

% set up table to collect data
dataTypes = ["double","double","double","double","double","double","double","double","double","double"];
dataLabels = ["time","PT1","PT2","PT3","PT4","LC5","LC6","LC7","FM","S1","S2","commandedState","DAQState","Queue Size"];
sz = [1,dataLength];
% testDataTable = table('Size',sz,'VariableTypes',dataTypes,'VariableNames',dataLabels);


% set up serial object
% serialPortName = 'COM5'; % on Windows would be COMx
serialPortName = '/dev/cu.SLAB_USBtoUART'; % Please don't delete this line--Hubert uses it for testing
s = serial(serialPortName,'BaudRate',115200);
% s = serialport(serialPortName,115200);
% open serial port
fopen(s);
% remember to fclose(s) in the command windows after ctrl+C exit the
% infinite while loop so that other programs can use the port

% set up plot
f = figure;
f.Position = [300 -100 800 650];
% t = tiledlayout(6,1);

% tileblock = []
% First tile
ax1 = nexttile;
ax1.XColor = [1 0 0];
ax1.YColor = [1 0 0];


% Second tile
ax2 = nexttile;
ax2.XColor = [1 0 0];
ax2.YColor = [1 0 0];


% Third tile
ax3 = nexttile;
ax3.XColor = [1 0 0];
ax3.YColor = [1 0 0];

% Fourth Tile
ax4 = nexttile;
ax4.XColor = [1 0 0];
ax4.YColor = [1 0 0];

% Fifth Tile
ax5 = nexttile;
ax5.XColor = [1 0 0];
ax5.YColor = [1 0 0];
%
% % Sixth Tile
% ax6 = nexttile;
% ax6.XColor = [1 0 0];
% ax6.YColor = [1 0 0];


timeControl = now();
% timeControlContinuous = now();
idx = 0;
idxSet = 0;

% for storing data sequentially in data1 and data2
i = 1;
% read data
flushinput(s);
% if there is not reading coming from the serial port, this command will
% result in a timeout before proceeding (default 10s, but the following
% command changes the time)
set(s, 'TimeOut', 2)
% fscanf(s);


% accounts for any time delay from reading
timeZeroer = 0;

strlengthWarned = false;


while (strlength(split(fscanf(s))) < 1)
    if strlengthWarned == false
        fprintf("No data/not enough data received through the serial port, check board\n")
    end
    strlengthWarned = true;

end

fprintf("Receiving data now\n")
errorLength = 0;
correctLength = 0;

while(1)
    %     startTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    %     startTime=startTime(21:23);
    
    rawStr = fscanf(s);
    linestr = split(rawStr,'←↵');
    str = split(linestr{1}," ");
    if length(str) ~= dataLength

        errorLength = errorLength + 1

        str
        rawStr
        length(str)


        continue;
    else
        correctLength = correctLength + 1;
    end


    % each data line represents one sensor data
    data(i,1) = (str2double(str{1})-timeZeroer)/1000;
    %     timeInterval(i) = (str2double(str{1})-timeZeroer)/1000;
    if i == 1
        timeZeroer = str2double(str{1});
        data(i,1) = (str2double(str{1})-timeZeroer)/1000;
    end

    for n = 2:dataLength

        data(i,n) = str2double(str{n})*calibrationData(n,1)+calibrationData(n,2);

        if i >=2 && i <=8
            data(1,n) = data(1,n)/100;
        end

    end
      data(i,12)

%     fprintf(data(:,))



    %     testDataTable(i,:) = {timeInterval(i),data1(i),data2(i),data3(i),data4(i),data5(i),data6(i),data7(i),data8(i)};





    %
    if (now() - timeControl) * 24 * 60 * 60 >= plotTime % plot every x seconds
        if data(end,1)-data(1,1) < observationInterval




            axes(ax1);
            % subplot(2,3,1),
            plot(data(:,1),data(:,2));
            title('Pressure Transducer 1')

            xlim([data(i,1)-observationInterval, data(i,1)]);

            axes(ax2);
            % subplot(2,3,2),
            plot(data(:,1),data(:,3));
            title('Pressure Transducer 2')

            xlim([data(i,1)-observationInterval, data(i,1)]);
            % ylim([-500000 100000]);

            axes(ax3);
            % subplot(2,3,3),
            plot(data(:,1),data(:,4));
            title('Pressure Transducer 3')

            xlim([data(i,1)-observationInterval, data(i,1)]);
            axes(ax4);
            % subplot(2,3,4),
            plot(data(:,1),data(:,5));
            title('Pressure Transducer 4')

            
                        axes(ax5);
            % subplot(2,3,5),
            plot(data(:,1),data(:,6)+data(:,7)+data(:,8));
                        title('Load Cells Combined')
            
 xlim([data(i,1)-observationInterval, data(i,1)]);            %
            %             axes(ax6);
            % % subplot(2,3,6),
            % plot(timeInterval,data8);
            %             title('FM');
            %
            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);

        else % plot only the latest 5 seconds of data
            %             timeInterval(i-1)-timeInterval(1) < observationInterval
            %             timeInterval(i)
            %             timeInterval(i)-timeInterval(1) >= observationInterval
            if idxSet == 0
                idx =  floor(length(data(:,1)))-1;
                idxSet = 1;

                %                 idx
                %             idxDumpTime = 1/(floor(length(timeInterval)/observationalInterval))
            end


            %             if (now() - timeControlContinuous) * 24 * 60 * 60 >= idxDumpTime
            %             idx

            axes(ax1);
            % subplot(2,3,1),
            plot(data(end-idx:end,1),data(end-idx:end,2));
            % set the x limits so that only the last 5 seconds of data is
            % plotted
            title('Pressure Transducer 1')

            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            xlim([data(i-idx,1), data(i,1)]);

            axes(ax2);
            % subplot(2,3,2),
            plot(data(end-idx:end,1),data(end-idx:end,3));
            title('Pressure Transducer 2')

            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);
%             xlim([data(i,1)-observationInterval, data(i,1)]);
 xlim([data(i-idx,1), data(i,1)]);

            axes(ax3);
            % subplot(2,3,3),
            plot(data(end-idx:end,1),data(end-idx:end,4));
            title('Pressure Transducer 3')

            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            % ylim([-500000 100000]);
%             xlim([data(i,1)-observationInterval, data(i,1)]);
 xlim([data(i-idx,1), data(i,1)]);


            axes(ax4);
            % subplot(2,3,4),
            plot(data(end-idx:end,1),data(end-idx:end,5));
            title('Pressure Transducer 4')

            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
%             xlim([data(i,1)-observationInterval, data(i,1)]);
 xlim([data(i-idx,1), data(i,1)]);




            
                        axes(ax5);
            LoadCellDataLastCombined = data(:,6)+data(:,7)+data(:,8);
%             subplot(2,3,5), 
plot(data(end-idx:end,1),LoadCellDataLastCombined(end-idx:end));
                        % set the x limits so that only last 5 seconds of data is
                        % plotted
                        title('Load Cells Total')
            
 xlim([data(i-idx,1), data(i,1)]);            %
            % %             axes(ax6);
            % subplot(2,3,6), PLOT(timeInterval(end-idx:end),data8(end-idx:end));
            %             title('FM')
            %
            %             xlim([timeInterval(i)-observationInterval, timeInterval(i)]);
            %
            %
            % %             timeControlContinuous = now();
            % %             end
        end
        timeControl = now();
    end
    %     endTime=datestr(now,'dd-mm-yyyy HH:MM:SS FFF');
    %     endTime=endTime(21:23);

    %     timeDifference=str2double(endTime)-str2double(startTime);
    %     if timeDifference<pauseTime
    %         pause(timeDifference);
    %     end
    i = i+1;
end
    function cleanMeUp()
        testDataTable = array2table(data,'VariableNames',dataLabels);
        % saves data to file (or could save to workspace)
        fprintf('Saving test data as %s.xls\n',fileName);
        setUpTest(['Test_Data_',datestr(now,'yyyy-mm-dd')],fileName,testDataTable);
        %         writetable(testDataTable,fileName,"FileType","spreadsheet");
        %         fclose(s);
        ratio = errorLength/correctLength
        clear s
        instrreset;
    end



end

function setUpTest(folderName,fileName,testDataTable)
if ~exist(folderName, 'dir')
    mkdir(folderName);
    fprintf("Test data folder created\n");
else
    fprintf("Folder already exists\n")
end
writetable(testDataTable,fileName,"FileType","spreadsheet");
fileString = fileName + ".xls";
movefile(fileString,folderName);

end
