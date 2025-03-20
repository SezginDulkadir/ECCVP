clc;
clear all;

input_image = 'IMG_2182.png';
input_txt = 'intersections.txt';
cube_size = 0.8;
output_projected_points_txt = 'projected_points.txt';

centroids = read_points_from_txt(input_txt);
if isempty(centroids)
    error('Failed to read points from TXT file. Terminating algorithm.');
end

[f, cx, cy, theta_x, theta_y, theta_z, t] = initialize_camera_parameters();

[cube_points, edges] = define_cube_geometry(cube_size);

optimized_params = optimize_with_ransac(centroids, cube_points, edges, f, cx, cy, theta_x, theta_y, theta_z, t);

visualize_results(input_image, centroids, cube_points, edges, optimized_params, output_projected_points_txt);

function centroids = read_points_from_txt(input_txt)
    file_data = fileread(input_txt);
    lines = regexp(file_data, '\n', 'split');
    centroids = [];
    for i = 1:length(lines)
        line = strtrim(lines{i});
        if startsWith(line, 'intersection_')
            coords = regexp(line, '(\d+\.\d+),\s*(\d+\.\d+)', 'tokens');
            if ~isempty(coords)
                coords = str2double(coords{1});
                centroids = [centroids; coords];
            end
        end
    end
end

function [f, cx, cy, theta_x, theta_y, theta_z, t] = initialize_camera_parameters()
    f = 2000;
    cx = 1290.1430;
    cy = 1800.5314;
    theta_x = deg2rad(0.5);
    theta_y = deg2rad(-0.51);
    theta_z = deg2rad(-0.52);
    t = [-1.1; -0.37; -2.65];
end

function [cube_points, edges] = define_cube_geometry(cube_size)
    reference_point = [0; 0; 0];
    p1 = reference_point;
    p2 = p1 + [0; cube_size; 0];
    p3 = p2 + [cube_size; 0; 0];
    p4 = p1 + [cube_size; 0; 0];
    p5 = p1 + [0; 0; cube_size];
    p6 = p2 + [0; 0; cube_size];
    p7 = p3 + [0; 0; cube_size];
    p8 = p4 + [0; 0; cube_size];
    cube_points = [p1, p2, p3, p4, p5, p6, p7, p8];
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
end

function optimized_params = optimize_with_ransac(centroids, cube_points, edges, f, cx, cy, theta_x, theta_y, theta_z, t)
    x0 = [f, cx, cy, theta_x, theta_y, theta_z, t'];
    lb = [300, 50, 50, -2*pi, -2*pi, -2*pi, -5, -5, -15];
    ub = [2500, 2500, 2500, 2*pi, 2*pi, 2*pi, 5, 5, -0.5];
    num_iterations = 500;
    best_params = x0;
    best_error = inf;
    for i = 1:num_iterations
        sampled_centroids = centroids(randsample(size(centroids, 1), min(5, size(centroids, 1))), :);
        objective_function = @(x) compute_error(x, cube_points, edges, sampled_centroids);
        options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off');
        params = fmincon(objective_function, x0, [], [], [], [], lb, ub, [], options);
        error = compute_error(params, cube_points, edges, centroids);
        if error < best_error
            best_error = error;
            best_params = params;
        end
    end
    optimized_params = best_params;
end

function error = compute_error(x, cube_points, edges, centroids)
    delta = 0.01;
    f = x(1);
    cx = x(2);
    cy = x(3);
    theta_x = x(4);
    theta_y = x(5);
    theta_z = x(6);
    t = x(7:9);
    K = [f 0 cx; 0 f cy; 0 0 1];
    R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
    R_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];
    R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
    R = R_z * R_y * R_x;
    P = K * [R t(:)];
    cube_points_h = [cube_points; ones(1, size(cube_points, 2))];
    projected_points_h = P * cube_points_h;
    projected_points = projected_points_h(1:2, :) ./ projected_points_h(3, :);
    [idx, d] = knnsearch(projected_points', centroids, 'K', 1);
    matched_points = projected_points(:, idx);
    residuals = vecnorm(centroids - matched_points', 2, 2);
    huber_loss = arrayfun(@(r) (r <= delta) * 0.5 * r^2 + (r > delta) * delta * (r - 0.5 * delta), residuals);
    error = mean(huber_loss);
end

function visualize_results(input_image, centroids, cube_points, edges, optimized_params, output_projected_points_txt)
    image = imread(input_image);
    f = optimized_params(1);
    cx = optimized_params(2);
    cy = optimized_params(3);
    theta_x = optimized_params(4);
    theta_y = optimized_params(5);
    theta_z = optimized_params(6);
    t = optimized_params(7:9);
    K = [f 0 cx; 0 f cy; 0 0 1];
    R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
    R_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];
    R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
    R = R_z * R_y * R_x;
    P = K * [R t(:)];
    cube_points_h = [cube_points; ones(1, size(cube_points, 2))];
    projected_points_h = P * cube_points_h;
    projected_points = projected_points_h(1:2, :) ./ projected_points_h(3, :);
    figure;
    imshow(image); hold on;
    plot(centroids(:, 1), centroids(:, 2), 'r+', 'MarkerSize', 20, 'LineWidth', 2);
    for i = 1:size(edges, 1)
        pt1 = projected_points(:, edges(i, 1));
        pt2 = projected_points(:, edges(i, 2));
        plot([pt1(1) pt2(1)], [pt1(2) pt2(2)], 'b-', 'LineWidth', 2);
    end
    hold off;
    saveas(gcf, 'visualized_result.png');
end
