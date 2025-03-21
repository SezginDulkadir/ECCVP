clc;
clear all;
tic;

frame = ones(480, 640, 3);

A = dlmread('Blinking_Led_320_undistorted.csv');

EventsPerFrame = 1000;

numEvents = size(A, 1);

outputDir = 'C:\Users\Assistt\Desktop\Vanishing Points Camera Calibration\Event Dataset\Event_Camera_Focal_Length_3\Blinking_Led_Experiment_Datasets\Blinking_Led_Without_Accumulator_3\Blinking_Led_320_Undistorted_Images';
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

for i = 1:EventsPerFrame:numEvents
    frame = ones(480, 640, 3);
    
    endIndex = min(i + EventsPerFrame - 1, numEvents);
    
    for j = i:endIndex
        x = A(j, 2);
        y = A(j, 3);
        p = A(j, 4);
        
        if x > 0 && x <= 640 && y > 0 && y <= 480
            if p == 0
                frame(y, x, :) = [0, 0, 255];
            elseif p == 1
                frame(y, x, :) = [255, 0, 0];
            end
        end
    end
    
    file_name = sprintf('Frame%d.png', ceil(i / EventsPerFrame));
    fullFileName = fullfile(outputDir, file_name);
    imwrite(frame, fullFileName, 'png');
end

toc;
