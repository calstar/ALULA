% Which .xls file are you reading from
fileName = 'PTCalibration_1';

% Name the sensors
testDevice = 'PT ';

table = readtable([fileName,'.xls']);
dataArray = table2array(table);
graphArray = dataArray(3:end,:);

solution = dataArray(1:2,1:end-1);

figure;
set(gcf, 'PaperSize', [10 10]);
plotNumber = length(graphArray(1,:))-1;

for k = 1:plotNumber
    nexttile
    sortedArray = sort(graphArray(:,k));

    x = linspace(sortedArray(1),sortedArray(end),100);
    titleString = [testDevice,num2str(k)];
    plot(graphArray(:,k),graphArray(:,end),'o')
    title(titleString);
    hold on
    y = solution(1,k)*x+solution(2,k);

    plot(x,y)
    hold off
end