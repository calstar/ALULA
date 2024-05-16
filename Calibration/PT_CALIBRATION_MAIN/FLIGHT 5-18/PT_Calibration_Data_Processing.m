fileext = '.csv';
filename = 'CORE_BOARD_A_WITHOUT_BATTERY_hx06_repinned_GX_connx2024-05-14_test1';
data = readtable(strcat(filename, fileext));
filename = 'CORE_BOARD_A_WITHOUT_BATTERY2024-05-13_test1';
data2 = readtable(strcat(filename, fileext));


%% 

tiledlayout(3,2)
names = ["O1" "Unused" "E1" "E2" "C1" "O2"];
format long;

for i = 1:6
    nexttile
    hold on;
    disp(strcat(names(i),": "))
    plot_and_fit(data, "poly2", i);
    scatter_data(data2, i);
    xlabel('Raw PT Output');
    ylabel('Pressure (psi)');
    title(names(i))
end

%%

function plot_and_fit(data, fittype, i)
    xdata = table2array(data(:,[strcat("Reading",int2str(i),"X")]));
    ydata = table2array(data(:,[strcat("Reading",int2str(i),"Y")]));
    f = fit(xdata, ydata, fittype)
    disp(coeffvalues(f))
    plot(f, xdata, ydata)
end

function scatter_data(data, i)
    xdata = table2array(data(:,[strcat("Reading",int2str(i),"X")]));
    ydata = table2array(data(:,[strcat("Reading",int2str(i),"Y")]));
    scatter(xdata, ydata);
end