function main()
    % Read points from the text file
    points = read_points_from_txt('projected_points.txt');
    
    % Actual K matrix
    K_actual = [778.0243, 0, 303.7560; 0, 780.0176, 256.3121; 0, 0, 1.0];
    
    % Compute K matrix from points
    [K, error] = compute_K_from_points(points, K_actual);
    
    % Display results
    disp('Computed K Matrix:');
    disp(K);
    disp('Error (Frobenius norm):');
    disp(error);
    disp('Actual K Matrix:');
    disp(K_actual);
end

function points = read_points_from_txt(file_path)
    points = containers.Map('KeyType', 'int32', 'ValueType', 'any');
    fid = fopen(file_path, 'r');
    line_number = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if line_number == 0
            line_number = line_number + 1;
            continue;
        end
        if ~isempty(line)
            parts = strsplit(line, ',');
            if length(parts) == 3
                point_id = str2double(parts{1});
                x = str2double(parts{2});
                y = str2double(parts{3});
                points(point_id) = [x, y];
            end
        end
        line_number = line_number + 1;
    end
    fclose(fid);
end

function [a, b, c] = compute_line_equation(points)
    pt1 = points(1, :);
    pt2 = points(2, :);
    slope = (pt2(2) - pt1(2)) / (pt2(1) - pt1(1));
    intercept = pt2(2) - slope * pt2(1);
    a = -slope;
    b = 1.0;
    c = -intercept;
end

function intersection_point = compute_point_of_intersection(line1, line2)
    a1 = line1(1);
    b1 = line1(2);
    c1 = line1(3);
    a2 = line2(1);
    b2 = line2(2);
    c2 = line2(3);
    x = (-c2 + c1) / (-a1 + a2);
    y = (-a1 * x) - c1;
    intersection_point = [x, y];
end

function vanishing_point = compute_vanishing_point(points)
    [a1, b1, c1] = compute_line_equation(points(1:2, :));
    [a2, b2, c2] = compute_line_equation(points(3:4, :));
    vanishing_point = compute_point_of_intersection([a1, b1, c1], [a2, b2, c2]);
    assert(dot([vanishing_point, 1.0], [a1, b1, c1]) == 0.0, 'Dot product of point and line must be zero');
end

function K = compute_K_from_vanishing_points(vanishing_points)
    A = [];
    for i = 1:length(vanishing_points)
        for j = i+1:length(vanishing_points)
            point_i = [vanishing_points{i}(1), vanishing_points{i}(2), 1.0];
            point_j = [vanishing_points{j}(1), vanishing_points{j}(2), 1.0];
            A = [A; point_i(1)*point_j(1) + point_i(2)*point_j(2), ...
                      point_i(1)*point_j(3) + point_i(3)*point_j(1), ...
                      point_i(2)*point_j(3) + point_i(3)*point_j(2), ...
                      point_i(3)*point_j(3)];
        end
    end
    [~, ~, V] = svd(A);
    w1 = V(1, end);
    w4 = V(2, end);
    w5 = V(3, end);
    w6 = V(4, end);
    w = [w1, 0., w4; 0., w1, w5; w4, w5, w6];
    K_transpose_inv = chol(w);
    K = inv(K_transpose_inv');
    K = K / K(end, end);
end

function [K, error] = compute_K_from_points(points, K_actual)
    v1 = compute_vanishing_point([points(8); points(5); points(7); points(6)]);
    v2 = compute_vanishing_point([points(1); points(2); points(5); points(6)]);
    v3 = compute_vanishing_point([points(8); points(4); points(5); points(1)]);
    vanishing_points = {v1, v2, v3};
    K = compute_K_from_vanishing_points(vanishing_points);
    error = norm(K - K_actual, 'fro');
end
