filename = 'CORE_BOARD_A_WITHOUT_BATTERY_hx06_repinned_GX_connx2024-05-14_test1';
fileext = '.csv';
data = readtable(strcat(filename, fileext));


%% 

tiledlayout(3,2)
names = ["O1" "Unused" "E1" "E2" "C1" "O2"];

for i = 1:6
    nexttile
    xdata = table2array(data(:,[strcat("Reading",int2str(i),"X")]));
    ydata = table2array(data(:,[strcat("Reading",int2str(i),"Y")]));
    disp(names(i))
    f = fit(xdata, ydata, 'poly2')
    plot(f, xdata, ydata)
    xlabel('Raw PT Output');
    ylabel('Pressure (psi)');
    title(names(i))
end