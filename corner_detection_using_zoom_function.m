clc;
clear all;

imageFile = 'IMG_2182.png';
img = imread(imageFile);
figure;
imshow(img);
title('Draw a Line on the Image');
axis on;
hold on;

lines = {};
intersections = struct();
markerSize = 10;
threshold = 15;

lineCount = 0;
intersectionCount = 0;

while true
    zoom on;
    disp('In zoom mode, use the mouse to zoom in and out. Press Enter to select a line.');
    pause;
    zoom off;
    
    lineCount = lineCount + 1;
    disp(['Select two points for line ', num2str(lineCount), ' (Press Enter to cancel).']);
    [x, y, button] = ginput(2);
    
    if isempty(x) || isempty(y) || length(x) ~= 2 || length(y) ~= 2
        disp('Drawing terminated.');
        break;
    end
    
    if isequal([x(1), y(1)], [x(2), y(2)])
        disp('Two points cannot be the same, please select different points.');
        continue;
    end
    
    if ~isFarFromExistingPoints([x, y], intersections, threshold)
        disp('The selected points are too close to existing intersection points, please select different points.');
        continue;
    end
    
    newLine = plot([x(1), x(2)], [y(1), y(2)], 'LineWidth', 2, 'Color', 'b');
    lines{end+1} = [x(1), y(1); x(2), y(2)];
    
    for i = 1:lineCount-1
        x1 = lines{i}(1, 1); y1 = lines{i}(1, 2);
        x2 = lines{i}(2, 1); y2 = lines{i}(2, 2);
        
        A1 = y1 - y2;
        B1 = x2 - x1;
        C1 = A1 * x1 + B1 * y1;
        
        A2 = y(1) - y(2);
        B2 = x(2) - x(1);
        C2 = A2 * x(1) + B2 * y(1);
        
        determinant = A1 * B2 - A2 * B1;
        
        if determinant ~= 0
            x_intersect = (B2 * C1 - B1 * C2) / determinant;
            y_intersect = (A1 * C2 - A2 * C1) / determinant;
            
            if isPointOnSegment([x1, y1], [x2, y2], [x_intersect, y_intersect]) && ...
               isPointOnSegment([x(1), y(1)], [x(2), y(2)], [x_intersect, y_intersect])
                
                intersectionCount = intersectionCount + 1;
                plot(x_intersect, y_intersect, 'go', 'MarkerSize', markerSize, 'LineWidth', 2);
                text(x_intersect + 5, y_intersect, num2str(intersectionCount), 'Color', 'yellow', 'FontSize', 36);
                
                intersections.(sprintf('intersection_%d', intersectionCount)) = [x_intersect, y_intersect];
                assignin('base', sprintf('intersection_%d', intersectionCount), [x_intersect, y_intersect]);
                
                disp(['Intersection point ', num2str(intersectionCount), ':']);
                disp([x_intersect, y_intersect]);
            end
        end
    end
end

hold off;

[folderPath, ~, ~] = fileparts(imageFile);
outputFile = fullfile(folderPath, 'intersections.txt');
fid = fopen(outputFile, 'w');

fprintf(fid, 'Intersection Points (x, y):\n');
intersectionFields = fieldnames(intersections);
for i = 1:length(intersectionFields)
    intersectionData = intersections.(intersectionFields{i});
    fprintf(fid, '%s: %.2f, %.2f\n', intersectionFields{i}, intersectionData(1), intersectionData(2));
end

fclose(fid);
disp(['Intersection points saved to "', outputFile, '".']);

function isOnSegment = isPointOnSegment(p1, p2, pt)
    isOnSegment = pt(1) >= min(p1(1), p2(1)) && pt(1) <= max(p1(1), p2(1)) && ...
                  pt(2) >= min(p1(2), p2(2)) && pt(2) <= max(p1(2), p2(2));
end

function isFar = isFarFromExistingPoints(newPoints, intersections, threshold)
    isFar = true;
    if isempty(fieldnames(intersections))
        return;
    end
    for i = 1:length(intersections)
        existingPoint = intersections.(sprintf('intersection_%d', i));
        if norm(newPoints(1, :) - existingPoint) < threshold || norm(newPoints(2, :) - existingPoint) < threshold
            isFar = false;
            break;
        end
    end
end
