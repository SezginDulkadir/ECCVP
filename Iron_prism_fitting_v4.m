%% --- NESNE POZİSYONU VE BOYUT TAHMİNİ ALGORİTMASI (HATA GİDERİLMİŞ) ---
% Açıklama: Bu script, bir görüntüde tespit edilen 2D köşe noktalarını kullanarak
% bir 3D prizmanın pozisyonunu (rotasyon, öteleme), kamera içsel parametrelerini (f, cx, cy)
% ve prizmanın boyutlarını (genişlik, derinlik, yükseklik) optimize eder.
%
% Yöntem: Algoritma, eksik köşe noktaları (occlusion) durumunu ele alacak şekilde
% tasarlanmıştır. Görüntüdeki her bir köşe noktası, 3D modelin projekte edilmiş
% 8 köşesinden kendisine en yakın olanla eşleştirilir. Hata, sadece bu
% eşleşmeler üzerinden hesaplanır. Bu, görünmeyen köşelerin optimizasyonu
% bozmamasını sağlar. Global optimumu bulma şansını artırmak için çoklu başlangıçlı
% (multi-start) bir optimizasyon stratejisi kullanılır.
%--------------------------------------------------------------------------

clc;
clear all;
close all;

%% 1. GİRİŞ PARAMETRELERİ
input_txt_file = 'intersections.txt'; % Tespit edilen 2D köşe noktalarını içeren dosya
background_image_file = 'Iron_300.png'; % Görselleştirme için arka plan görüntüsü

%% 2. BAŞLANGIÇ DEĞERLERİ VE OPTİMİZASYON SINIRLARI
% Bu değerler, optimizasyonun başlayacağı kaba tahminlerdir.
f_initial = 717.88;
cx_initial = 250.00;
cy_initial = 623.16;
theta_x_initial = 1.05;
theta_y_initial = -0.81;
theta_z_initial = 1.51;
t_initial = [-0.34; 2.09; -4.67];
initial_dimensions = [0.80, 1.41, 0.80]; % [width, depth, height]

% Optimizasyon değişkenleri için bir başlangıç vektörü oluştur
% Format: [f, cx, cy, theta_x, theta_y, theta_z, t_x, t_y, t_z, w, d, h]
x0_initial = [f_initial, cx_initial, cy_initial, theta_x_initial, theta_y_initial, theta_z_initial, t_initial', initial_dimensions];

% Optimizasyon parametreleri için alt ve üst sınırlar
% Bu sınırlar, arama uzayını mantıklı bir aralıkta tutar.
lower_bounds = [500,  250,  200, -pi/2, -pi/2, -pi/2, -3, -3, -6, 0.8, 0.3, 0.2];
upper_bounds = [1300, 800,  800,  pi/2,  pi/2,  pi/2,  3,  3, -1, 3.0, 1.5, 0.8];

%% 3. ALGORITMA YÜRÜTME
% Görüntüdeki köşe noktalarını dosyadan yükle
image_points = load_points_from_file(input_txt_file);

% Çoklu başlangıç noktası ile robust optimizasyonu çalıştır
[optimized_params, final_error, final_residuals] = run_robust_optimization(image_points, x0_initial, lower_bounds, upper_bounds);

% Sonuçları görselleştir
visualize_results(image_points, optimized_params, background_image_file, final_residuals);

%% --- FONKSİYONLAR ---

function centroids = load_points_from_file(file_path)
    % Verilen text dosyasından köşe noktası koordinatlarını (x, y) okur.
    if exist(file_path, 'file') ~= 2
        error('Giriş noktalarını içeren dosya bulunamadı: %s', file_path);
    end
    raw_data = readlines(file_path);
    centroids = [];
    for i = 1:length(raw_data)
        tokens = regexp(raw_data(i), '(\d+\.?\d*),\s*(\d+\.?\d*)', 'tokens');
        if ~isempty(tokens)
            coords = str2double(tokens{1});
            centroids = [centroids; coords];
        end
    end
    if isempty(centroids)
        error('Giriş dosyasında geçerli formatta nokta bulunamadı.');
    end
    fprintf('%d adet köşe noktası başarıyla yüklendi.\n', size(centroids, 1));
end

function [prism_points, edges] = define_prism_geometry(dimensions)
    % Verilen boyutlara göre bir prizmanın 8 köşe noktasını ve 12 kenarını tanımlar.
    width = dimensions(1); 
    depth = dimensions(2); 
    height = dimensions(3);
    
    % Prizmanın köşe noktaları (orijin merkezli)
    w_half = width / 2; d_half = depth / 2; h_half = height / 2;
    p = [ -w_half, -d_half, -h_half; ...
           w_half, -d_half, -h_half; ...
           w_half,  d_half, -h_half; ...
          -w_half,  d_half, -h_half; ...
          -w_half, -d_half,  h_half; ...
           w_half, -d_half,  h_half; ...
           w_half,  d_half,  h_half; ...
          -w_half,  d_half,  h_half ];
    prism_points = p'; % Transpozunu alarak (3x8) formatına getir
      
    edges = [1 2; 2 3; 3 4; 4 1; 5 6; 6 7; 7 8; 8 5; 1 5; 2 6; 3 7; 4 8];
end

function [error_score, full_residuals] = compute_reprojection_error(params, image_points)
    delta = 1;
    f = params(1); cx = params(2); cy = params(3);
    theta_x = params(4); theta_y = params(5); theta_z = params(6);
    t = params(7:9);
    dimensions = params(10:12);
    
    [object_points, ~] = define_prism_geometry(dimensions);
    
    K = [f 0 cx; 0 f cy; 0 0 1];
    R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
    R_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];
    R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
    R = R_z * R_y * R_x;
    
    % *** HATA DÜZELTİLDİ: R(:) -> R ***
    P = K * [R, t(:)];
    
    object_points_h = [object_points; ones(1, size(object_points, 2))];
    projected_points_h = P * object_points_h;
    projected_points = projected_points_h(1:2, :) ./ projected_points_h(3, :);
    
    [matched_indices, distances] = knnsearch(projected_points', image_points, 'K', 1);
    residuals = distances;
    
    huber_loss = arrayfun(@(r) (r <= delta) * 0.5 * r^2 + (r > delta) * delta * (r - 0.5 * delta), residuals);
    error_score = mean(huber_loss);
    
    full_residuals = zeros(8, 1);
    unique_indices = unique(matched_indices);
    for i = 1:length(unique_indices)
        idx = unique_indices(i);
        avg_dist = mean(distances(matched_indices == idx));
        full_residuals(idx) = avg_dist;
    end
end

function [best_params, best_error, final_residuals] = run_robust_optimization(image_points, x0, lb, ub)
    num_iterations = 500;
    best_params_so_far = x0;
    best_error = inf;
    
    fprintf('Robust optimizasyon başlıyor... %d rastgele deneme yapılacak.\n', num_iterations);
    
    options = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'off', 'TolFun', 1e-7, 'TolX', 1e-7);
    objective_function = @(x) compute_reprojection_error(x, image_points);

    for i = 1:num_iterations
        x0_random = lb + (ub - lb) .* rand(size(lb));
        
        [current_params, current_error] = fmincon(objective_function, x0_random, [], [], [], [], lb, ub, [], options);
        
        if current_error < best_error
            best_error = current_error;
            best_params_so_far = current_params;
            fprintf('Yeni en iyi hata: %.4f (Piksel) | Deneme: %d/%d\n', best_error, i, num_iterations);
        end
    end
    
    fprintf('\nEn iyi sonuç üzerinde son bir hassas ayar yapılıyor...\n');
    options.Display = 'final';
    [best_params, ~] = fmincon(objective_function, best_params_so_far, [], [], [], [], lb, ub, [], options);
    
    [best_error, final_residuals] = compute_reprojection_error(best_params, image_points);
    fprintf('\nOptimizasyon Tamamlandı!\nNihai Ortalama Reprojeksiyon Hatası (Huber): %.4f\n', best_error);
end

function visualize_results(image_points, optimized_params, bg_image_file, residuals)
    if exist(bg_image_file, 'file')
        img = imread(bg_image_file);
    else
        img_size = [max(image_points(:,2))+50, max(image_points(:,1))+50, 3];
        img = ones(round(img_size)) * 240;
        warning('Arka plan görüntüsü bulunamadı. Boş bir tuval kullanılıyor.');
    end

    figure('Name', 'Optimizasyon Sonucu', 'Position', [100, 100, 800, 600]);
    imshow(img);
    hold on;
    
    f = optimized_params(1); cx = optimized_params(2); cy = optimized_params(3);
    theta_x = optimized_params(4); theta_y = optimized_params(5); theta_z = optimized_params(6);
    t = optimized_params(7:9);
    dimensions = optimized_params(10:12);
    
    [object_points, edges] = define_prism_geometry(dimensions);
    
    K = [f 0 cx; 0 f cy; 0 0 1];
    R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
    R_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];
    R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
    R = R_z * R_y * R_x;
    
    % *** HATA DÜZELTİLDİ: R(:) -> R ***
    P = K * [R, t(:)];
    
    object_points_h = [object_points; ones(1, size(object_points, 2))];
    projected_points_h = P * object_points_h;
    projected_points = projected_points_h(1:2, :) ./ projected_points_h(3, :);
    
    plot(image_points(:, 1), image_points(:, 2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
    
    for i = 1:size(edges, 1)
        pt1 = projected_points(:, edges(i, 1));
        pt2 = projected_points(:, edges(i, 2));
        plot([pt1(1) pt2(1)], [pt1(2) pt2(2)], 'b-', 'LineWidth', 2);
    end
    
    plot(projected_points(1, :), projected_points(2, :), 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
    for i = 1:size(projected_points, 2)
        text(projected_points(1, i)+5, projected_points(2, i), sprintf('%d', i), ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    end
    
    title('Optimize Edilmiş Model ve Görüntü Noktaları');
    legend('Tespit Edilen Noktalar', 'Optimize Edilmiş Model');
    hold off;
    
    figure('Name', 'Reprojeksiyon Hata Dağılımı');
    bar(residuals);
    title('Her Bir Köşe Noktasının Reprojeksiyon Hatası');
    xlabel('Model Köşe Noktası Numarası');
    ylabel('Hata (Piksel)');
    grid on;
    xticks(1:8);
    xlim([0.5, 8.5]);
    ax = gca;
    ax.FontSize = 11;
end