function optimizeAndFindCorners()
% =========================================================================
% ANA OPTİMİZASYON FONKSİYONU - KALİTE SKORU TABANLI
% =========================================================================
% Bu fonksiyon, kullanıcıdan bir görüntü alır, tüm olası parametreleri
% dener, 6 köşe üreten tüm sonuçları saklar ve bunlar arasından kalite
% skoru (dışbükey kabuk alanı) en yüksek olanı "en iyi" sonuç olarak seçer.

% Kullanıcıdan Görüntü Dosya Yolunu Al
[filename, pathname] = uigetfile({'*.png;*.jpg;*.jpeg;*.tif'}, 'Lütfen bir görüntü dosyası seçin');
if isequal(filename, 0)
    disp('İşlem iptal edildi.');
    return;
end
img_path = fullfile(pathname, filename);

% ==================== Optimizasyon Parametre Aralıkları ====================
dist_threshold_range = 45:1:60;
fill_gap_range       = 6:1:10;
min_length_range     = 30:1:55;

% Tüm geçerli sonuçları saklamak için bir yapı (struct) dizisi
valid_solutions = [];
solution_count = 0;

fprintf('Optimizasyon süreci başlıyor...\n');
fprintf('Tüm parametreler denenecek ve 6 köşeli sonuçlar saklanacak.\n\n');

% Parametreler üzerinde döngü
for dist_thresh = dist_threshold_range
    for fill_gap = fill_gap_range
        for min_len = min_length_range
            
            fprintf('Denenen parametreler -> dist: %d, gap: %d, len: %d\n', dist_thresh, fill_gap, min_len);

            % Çekirdek işlem fonksiyonunu çağır
            [current_corners, extended_lines] = processImageWithParams(img_path, dist_thresh, fill_gap, min_len);
            
            % Eğer 6 köşe bulunduysa, listeye ekle
            if size(current_corners, 1) == 6
                solution_count = solution_count + 1;
                fprintf('>>> Geçerli bir aday bulundu! (Toplam: %d)\n', solution_count);
                
                % Bu adayın kalite skorunu hesapla (Metrik 1: Convex Hull Alanı)
                [~, score] = convhull(current_corners(:,1), current_corners(:,2));

                % Sonucu ve bilgilerini sakla
                valid_solutions(solution_count).corners = current_corners;
                valid_solutions(solution_count).extended_lines = extended_lines;
                valid_solutions(solution_count).score = score;
                valid_solutions(solution_count).params = struct('dist_thresh', dist_thresh, 'fill_gap', fill_gap, 'min_len', min_len);
            end
        end
    end
end

% ==================== En İyi Sonucun Seçilmesi ve Gösterilmesi ====================
if isempty(valid_solutions)
    fprintf('\nOptimizasyon süreci tamamlandı ancak 6 köşe veren bir sonuç bulunamadı.\n');
    return;
end

fprintf('\nToplam %d adet geçerli aday bulundu. En iyisi seçiliyor...\n', length(valid_solutions));

% Tüm skorları bir diziye topla ve en yüksek skorun indeksini bul
all_scores = [valid_solutions.score];
[~, best_index] = max(all_scores);

% En iyi sonucu değişkenlere ata
best_solution = valid_solutions(best_index);
best_corners = best_solution.corners;
best_params = best_solution.params;
best_extended_lines = best_solution.extended_lines;

fprintf('\nOptimizasyon Tamamlandı!\n');
fprintf('En iyi kalite skoruna (%.2f) sahip parametreler:\n', best_solution.score);
disp(best_params);

% Nihai sonucu görselleştir
img = imread(img_path);
figure('Name', 'Nihai Sonuç: Kalite Skoruna Göre En İyi Köşeler');
imshow(img); hold on;

% Uzatılmış çizgileri çiz
for k = 1:size(best_extended_lines, 1)
    plot([best_extended_lines(k,1) best_extended_lines(k,3)], [best_extended_lines(k,2) best_extended_lines(k,4)], 'g--', 'LineWidth', 1);
end

% Dışbükey kabuk çizgilerini çiz
if size(best_corners, 1) > 2
    K = convhull(best_corners(:,1), best_corners(:,2));
    convex_hull_points = best_corners(K,:);
    plot(convex_hull_points(:,1), convex_hull_points(:,2), 'm', 'LineWidth', 2);
end

% Nihai köşe noktalarını çiz
plot(best_corners(:,1), best_corners(:,2), 'r+', 'MarkerSize', 12, 'LineWidth', 2);
title(sprintf('En İyi Sonuç (Skor: %.2f)\ndist: %d, gap: %d, len: %d', ...
              best_solution.score, best_params.dist_thresh, best_params.fill_gap, best_params.min_len));
disp('Tespit Edilen En İyi Köşe Koordinatları (x, y):');
disp(best_corners);
hold off;

% Dosyaya kaydetme
try
    [script_folder, ~, ~] = fileparts(mfilename('fullpath'));
    if isempty(script_folder)
        script_folder = pwd;
    end
    output_filename = 'intersections.txt';
    full_path_to_file = fullfile(script_folder, output_filename);
    writematrix(best_corners, full_path_to_file);
    fprintf('\nKöşe noktaları şu dosyaya başarıyla kaydedildi:\n%s\n', full_path_to_file);
catch ME
    warning('Dosya kaydetme sırasında bir hata oluştu. Dosya geçerli MATLAB klasörüne kaydedilmeye çalışılıyor.');
    writematrix(best_corners, 'intersections.txt');
    fprintf('\nKöşe noktaları "intersections.txt" dosyasına başarıyla kaydedildi.\n');
    disp(ME.message);
end

end % <<<< ANA OPTİMİZASYON FONKSİYONUNUN BİTİŞİ


%% =========================================================================
% ÇEKİRDEK İŞLEM FONKSİYONU
% =========================================================================
function [final_corners, extended_lines] = processImageWithParams(img_path, dist_thresh, fill_gap, min_len)
% Bu fonksiyon, verilen parametrelerle görüntü işleme adımlarını gerçekleştirir
% ve tespit edilen köşe noktalarını döndürür.

% Görüntüyü oku
img = imread(img_path);
gray = im2gray(img);
[~, img_width, ~] = size(img);

% 1. Ön İşleme ve Bölütleme
blurred_gray = imgaussfilt(gray, 2);
threshold = graythresh(blurred_gray);
binary_mask = imbinarize(blurred_gray, threshold);
binary_mask = ~binary_mask;
binary_mask = bwareaopen(binary_mask, 200);
bw_edges = edge(binary_mask, 'canny');

% 2. Hough Çizgi Tespiti (Parametreler burada kullanılıyor)
[H, theta, rho] = hough(bw_edges);
P = houghpeaks(H, 10, 'threshold', ceil(0.3 * max(H(:))));
lines = houghlines(bw_edges, theta, rho, P, 'FillGap', fill_gap, 'MinLength', min_len);

% 3. Çizgileri Uzatma ve Kesişim Tespiti
all_intersections = [];
extended_lines = [];
if length(lines) > 1
    for i = 1:length(lines)
        p1 = lines(i).point1; p2 = lines(i).point2;
        m = (p2(2) - p1(2)) / (p2(1) - p1(1) + eps);
        c = p2(2) - m * p2(1);
        x1_ext = 1; y1_ext = m * x1_ext + c;
        x2_ext = img_width; y2_ext = m * x2_ext + c;
        extended_lines = [extended_lines; x1_ext, y1_ext, x2_ext, y2_ext];
    end
    for i = 1:size(extended_lines, 1)
        for j = i+1:size(extended_lines, 1)
            x1 = extended_lines(i, 1); y1 = extended_lines(i, 2);
            x2 = extended_lines(i, 3); y2 = extended_lines(i, 4);
            x3 = extended_lines(j, 1); y3 = extended_lines(j, 2);
            x4 = extended_lines(j, 3); y4 = extended_lines(j, 4);
            den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
            if abs(den) > 1e-6
                t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den;
                x = x1 + t*(x2-x1);
                y = y1 + t*(y2-y1);
                if isPointOnContour(x, y, bw_edges, 15)
                    all_intersections = [all_intersections; x, y];
                end
            end
        end
    end
end

% 4. Mesafe Tabanlı Filtreleme ve Nihai Köşe Seçimi
final_corners = [];
if isempty(all_intersections)
    return; % Kesişim yoksa boş döndür
end

% Mesafe tabanlı filtreleme (Parametre burada kullanılıyor)
final_corners = distanceFilterCorners(all_intersections, dist_thresh);

end


%% --- YARDIMCI FONKSİYONLAR ---
% Bu fonksiyonlar ana fonksiyonun dışında ama aynı .m dosyası içinde yer alır.

function isOnContour = isPointOnContour(x, y, mask, tolerance)
    isOnContour = false;
    if x < 1 || x > size(mask, 2) || y < 1 || y > size(mask, 1)
        return;
    end
    x_min = max(1, floor(x - tolerance));
    x_max = min(size(mask, 2), ceil(x + tolerance));
    y_min = max(1, floor(y - tolerance));
    y_max = min(size(mask, 1), ceil(y + tolerance));
    
    window = mask(y_min:y_max, x_min:x_max);
    if any(window(:))
        isOnContour = true;
    end
end

function unique_corners = distanceFilterCorners(corners, threshold)
    if isempty(corners)
        unique_corners = [];
        return;
    end
    
    % Köşeler arası uzaklık matrisini hesapla
    dist_matrix = squareform(pdist(corners));
    
    % Bağlantılı bileşenleri bulmak için graf oluştur
    adj_matrix = dist_matrix <= threshold;
    G = graph(adj_matrix);
    
    % Her bir bağlantılı bileşeni (küme) bul
    bins = conncomp(G);
    
    unique_corners = [];
    % Her bir kümenin ortalamasını (centroid) al
    for i = 1:max(bins)
        cluster_points = corners(bins == i, :);
        unique_corners = [unique_corners; mean(cluster_points, 1)];
    end
end