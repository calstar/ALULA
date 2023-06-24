%% Question 6
% Read video
flame = VideoReader("C:\Users\liams\Downloads\Hotfire IR.mp4");

FR = flame.FrameRate;

Frames = flame.NumFrames;
Height = flame.Height;
Width = flame.Width;
PixBits = flame.BitsPerPixel;

flameBits = Height*Width*PixBits*Frames;

%% ProcessPract
% Start here to get the right image processing before you start making
% loops. It's always a good idea to start small when coding!

% Find the first, last frame of the burn:
FlameStart = 895;
FlameEnd = 1044;
Frame1 = read(flame, FlameEnd); %mess about to dial in to st/fin frame
Frame1 = rgb2gray(Frame1); %create single spectrum for intensity analysis
imshow(Frame1);



imtool(Frame1); 
%^VERY NICE^ Use this for: single frame temp-intensity
%matching, pixel location for transient tracing


%% Process Temperature Field

% Check the maximum intensity in hotfire frame
max_intensity = 253; 
%CORRESPONDS TO 357*F according to video


% Mess around with values to calibrate intensity field with colorbar
Temp_357 = double(max_intensity); %max temp recorded on IR
Temp_310 = Temp_357*(0.8); %Match ColorBar to Threshold Temp in Original.
Temp_275 = Temp_357*(0.74);
Temp_250 = Temp_357*(0.685);
Temp_200 = Temp_357*(0.575);
Temp_175 = Temp_357*(0.493);
Temp_150 = Temp_357*(0.390);

Temp = [Temp_357 Temp_310 Temp_275 Temp_250 Temp_200 Temp_175 Temp_150; 357 310 275 250 200 175 150];

figure(1)
plot(Temp(1,:), Temp(2, :));
hold on
title("Calibration Plot");

%Calibration CurveFit
coeffs = polyfit(Temp(1,:),Temp(2, :),1)';
calfit = Temp(1,:).*coeffs(1)+coeffs(2);
plot(Temp(1,:), calfit);
hold off

lower_intensity = Temp_150/Temp_357; %Select Temp Threshold
upper_intensity = max_intensity; % set this value

% Binarize based on intensity thresholds
bwFrame1 = imbinarize(Frame1, lower_intensity);
figure(2)
imshow(bwFrame1);

%% Full Frame Processing
% Use this loop for tracking features across burn

% Pull frames between start and finish at an interval of X frames
frame_start = FlameStart;
frame_end = FlameEnd;
interval = 10;
flameframes = frame_start:interval:frame_end;
    

%Set up Hotspot Temp Tracing - use imtool to find general area to trace
HotSpot = Frame1((1010:1040), (540:570));
imshow(HotSpot);
%% 

% Loop to get all frames:
% Initialize
for i=1:length(flameframes)
    
    % Read the current frame
    FrameI = read(flame, flameframes(i));
    FrameI = rgb2gray(FrameI); 
    % Binarize based on intensity threshold
    bwFrameI = imbinarize(FrameI,lower_intensity);



    imshow(bwFrameI);
end
