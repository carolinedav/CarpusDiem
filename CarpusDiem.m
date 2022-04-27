%% CarpusDiem: Equine Carpus Lameness Diagnostic System
% Code written by Caroline Davis
% CarpusDiem is an interactive MATLAB script designed to aid carpal lameness 
% diagnostic testing for all experience levels of equine management. 
% CarpusDiem utilizes the Shi-Tomasi Corner Detection and a minimum eigenvalue 
% algorithm to detect trackable points from user input. Points are continuously 
% filtered against a forward-backward error threshold to maintain validity. 
% Valid points are used to calculate velocity, acceleration, force of movement, 
% and final net force acting on the carpus as a gait is maintained. 

%% instructions:
% please follow prompts below
% common errors:
% 1. video does not load
% solution: please go to apps > get more apps > download computer vision
% toolbox & image processing toolbox
% 2. array out of bounds error during analysis
% solution: the algorithm was unable to find enough points within your
% point-of-interest (POI). please rerun the script and select a better POI.

%% step 1: create video display for point-of-interest selection
video = input(' please input video file name with single quotes, otherwise press Ctrl + C to exit the program:  \n ');
fps = input(' please denote frames per second (fps) as a numerical value: \n helpful hint: iphones commonly use 30 fps \n ');
weight = input(' please enter an estimated weight for the horse in pounds (lbs): \n ');
tracking = input(" please denote the direction the horse is tracking from with single quotes:\n (i.e. 'left' or 'right' side of the video)\n ");
mass = weight / 2.205;

videoReader = VideoReader(video);
videoPlayer = vision.VideoPlayer('Position', [100,100,680,520]);
objectFrame = readFrame(videoReader);

%% step 2: allow interactive point-of-interest selection
% load first frame & request point-of-interest selection
figure;
imshow(objectFrame);
title('Please select point-of-interest approximately at the withers')
objectRegion = round(getPosition(imrect));
objectImage = insertShape(objectFrame, 'Rectangle', objectRegion, 'Color', 'red');
% show point-of-interest
figure;
imshow(objectImage);
title('Red box highlights point-of-interest');
% show detected images
points = detectMinEigenFeatures(rgb2gray(objectFrame), 'ROI', objectRegion);
pointImage = insertMarker(objectFrame, points.Location, '+', 'Color', 'white');
figure;
imshow(pointImage);
title('Detected points-of-interest')

%% step 3: instaniate tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1);
initialize(tracker, points.Location, objectFrame);

%% step 4: track point-of-interest in video
frameNum = 1;
pointCollection = {};
validityCollection = {};

while hasFrame(videoReader)
    frame = readFrame(videoReader);
    [points, validity] = tracker(frame);
    pointCollection{frameNum} = points;
    validityCollection{frameNum} = validity;
    out = insertMarker(frame, points(validity, :), '+');
    videoPlayer(out);
    frameNum = frameNum+1;
end

%% step 5: release the video
release(videoPlayer);
    
%% step 6: motion & mechanical behavior estimation
% select only valid points-of-interest in meters, assuming 1 pixel is 0.0002645833m
validPoints = {};
finalFrame = validityCollection{length(validityCollection)};
validFrames = zeros(length(finalFrame)-1,1);
m = 0.0002645833;
timeStep = fps*0.001;
for i = 1:frameNum-1
    for j = 1:length(finalFrame)
        if finalFrame(j) == 1
            validFrames(j) = j;
            validPoints{i}(j,:) = pointCollection{i}(j,:).*m;
        end
    end
end

% calculate velocity (meters per second), assuming equal time change between two consecutive valid points
velocity = {};
validVelocity = {};
for i = 1:frameNum-1
    for j = 1:length(validPoints{i})-1
        if validFrames(j) ~= 0 && validFrames(j+1) ~= 0
            validVelocity{i}(j,1) = j;
            velocity{i}(j,1) = (validPoints{i}(j+1,1) - validPoints{i}(j, 1)) / timeStep;
        end
    end
end

% calculate acceleration (meters per second-squared), velocity assumptions apply
acceleration = {};
validAcceleration = {};
for i = 1:frameNum-1
    for j = 1:length(velocity{i})-1
        if velocity{i}(j) ~= 0 && velocity{i}(j+1) ~= 0
            validAcceleration{i}(j,1) = j;
            acceleration{i}(j,1) = (velocity{i}(j+1,1) - velocity{i}(j, 1)) / timeStep;
        end
    end
end

% calculate force in kg*m/s^2
force = {};
validForce = {};
for i = 1:frameNum-1
    for j = 1:length(acceleration{i})
        if acceleration{i}(j) ~= 0
            validForce{i} = j;
            force{i} = acceleration{i}(j)*mass;
        end
    end
end 

% define force relationship
ks = 131;
b = 0.8;
forceOut = {};
concatXPosition = zeros((frameNum-1),1);
concatYPosition = zeros((frameNum-1),1);
for i = 1:frameNum-1
    for j = 1:length(force{i})
        if force{i}(j) ~= 0
            x = validPoints{i}(validForce{i},1);
            concatXPosition(i) = x;
            concatYPosition(i) = validPoints{i}(validForce{i},2);
            Dx = velocity{i}(validForce{i});
            forceOut{i} = force{i}(j) - ks*x - b*Dx;
        end
    end
end 

% plot horizontal position 
figure;
hold on;
t = transpose((1:frameNum-1)*(fps*0.001)); 
c = linspace(1,10,length(t)); 
if contains(tracking, 'right')
    scatter(t, flip(concatXPosition), [], c, 'filled');
end
if contains(tracking, 'left')
    scatter(t, concatXPosition, [], c, 'filled');
end
linearFit = lsline;
set(linearFit, 'color', '#487048', 'LineWidth', 1)
title('Horizontal position of detected points-of-interests')
xlabel('Time, seconds (s)')
ylabel('Position, meters (m)')
shg

% plot vertical position 
figure;
hold on;
t = transpose((1:frameNum-1)*(fps*0.001)); 
scatter(t, concatYPosition, [], c, 'filled');
[p, ~, mu] = polyfit(t, concatYPosition, 7);
yfit = polyval(p, t, [], mu);
plot(t, yfit, 'Color', '#487048', 'LineWidth', 1);
title('Vertical position of detected points-of-interests')
xlabel('Time, seconds (s)')
ylabel('Position, meters (m)')
shg

% plot input force 
figure;
hold on;
t = transpose((1:frameNum-1)*(fps*0.001));
concatForce = cat(1, force{:});
c = linspace(1,10,length(t)); 
scatter(t, concatForce, [], c);
[p, ~, mu] = polyfit(t, concatForce, 7);
yfit = polyval(p, t, [], mu);
plot(t, yfit, 'Color', '#E5721D', 'LineWidth', 1);
title('Measured force exerted on leg')
xlabel('Time, seconds (s)')
ylabel('Force, Newtons (N or kg*m/s^2)')
shg

% plot output force 
figure;
hold on;
m = 0.0002645833;
t = transpose((1:frameNum-1)*(fps*0.001));
concatForceOut = cat(1, forceOut{:});
c = linspace(1,10,length(t)); 
scatter(t, concatForceOut, [], c);
[p, ~, mu] = polyfit(t, concatForceOut, 7);
yfit = polyval(p, t, [], mu);
plot(t, yfit, 'Color', '#2F243A', 'LineWidth', 1);
title('Resulting force exerted on carpus bone')
xlabel('Time, seconds (s)')
ylabel('Force, Newtons (N or kg*m/s^2)')
shg


