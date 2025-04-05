clc;
clear all;

input_image = 'Blinking_Led_300_Without_Accumulator.png';
output_txt = 'image_points.txt';
output_projected_points_txt = 'projected_points.txt';
cube_size = 0.9;

centroids = detect_dense_points(input_image, output_txt);
if isempty(centroids)
    error('Image points could not be detected. Terminating the algorithm.');
end

[f, cx, cy, theta_x, theta_y, theta_z, t] = initialize_camera_parameters();

[cube_points, edges] = define_cube_geometry(cube_size);

optimized_params = optimize_with_ransac(centroids, cube_points, edges, f, cx, cy, theta_x, theta_y, theta_z, t);

visualize_results(input_image, centroids, cube_points, edges, optimized_params, output_projected_points_txt);

function [f, cx, cy, theta_x, theta_y, theta_z, t] = initialize_camera_parameters()
    f = 718;
    cx = 359.1430;
    cy = 410.5314;
    theta_x = deg2rad(0.51);
    theta_y = deg2rad(0.18);
    theta_z = deg2rad(0.13);
    t = [-0.39; 0.33; -2.52];
end

function centroids = detect_dense_points(input_image, output_txt)
    image = imread(input_image);

    if size(image, 3) == 3
        blueChannel = image(:, :, 3);
        redChannel = image(:, :, 1);
        greenChannel = image(:, :, 2);
    else
        error('Image must be colored.');
    end

    blueMask = (blueChannel > 100) & (blueChannel > redChannel) & (blueChannel > greenChannel);

    cleanMask = bwareaopen(blueMask, 10);
    cc = bwconncomp(cleanMask);
    regionProps = regionprops(cc, 'Centroid');

    centroids = cat(1, regionProps.Centroid);

    if isempty(centroids)
        disp('No dense centers detected.');
        return;
    end

    epsilon = 500;
    minPoints = 100;
    clusterLabels = dbscan(centroids, epsilon, minPoints);

    validCentroids = centroids(clusterLabels > 0, :);

    if isempty(validCentroids)
        disp('No dense clusters found.');
        return;
    end
    writematrix(validCentroids, output_txt, 'Delimiter', 'tab');
    disp(['Dense blue points saved to ', output_txt, ' file.']);
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
    projected_points;
end

function optimized_params = optimize_with_ransac(centroids, cube_points, edges, f, cx, cy, theta_x, theta_y, theta_z, t)
    x0 = [f, cx, cy, theta_x, theta_y, theta_z, t'];
    lb = [500, 250, 200, -pi, -pi, -pi, -2, -2, -10];
    ub = [1300, 650, 650, pi, pi, pi, 2, 2, -1];
    
    num_iterations = 50;
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
    box on;
    imshow(image); hold on;
    plot(centroids(:, 1), centroids(:, 2), 'r+', 'MarkerSize', 20, 'LineWidth', 2);
    for i = 1:size(edges, 1)
        pt1 = projected_points(:, edges(i, 1));
        pt2 = projected_points(:, edges(i, 2));
        plot([pt1(1) pt2(1)], [pt1(2) pt2(2)], 'b-', 'LineWidth', 2);
    end
    for i = 1:size(projected_points, 2)
        text(projected_points(1, i), projected_points(2, i), sprintf('%d', i), 'Color', 'yellow', 'FontSize', 20, 'FontWeight', 'bold');
    end
    hold off;
    saveas(gcf, 'visualized_result.png');

    fid = fopen(output_projected_points_txt, 'w');
    fprintf(fid, 'ID, X, Y\n');
    for i = 1:size(projected_points, 2)
        fprintf(fid, '%d, %.4f, %.4f\n', i, projected_points(1, i), projected_points(2, i));
    end
    fclose(fid);
    disp(['Projected points saved to ', output_projected_points_txt, ' file.']);
end
