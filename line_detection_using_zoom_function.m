clc;
clear all;

% 1. Görüntüyü yükle ve göster
imageFile = 'IMG_2182.png'; % Kendi görselinizin yolunu girin
img = imread(imageFile);

% Görüntüyü döndürmek için bir açı belirleyin (örneğin, 45 derece)
% rotationAngle = 90; % Pozitif değer saat yönünde, negatif değer saat yönünün tersine döner

% Görüntüyü döndür
% img = imrotate(img, rotationAngle);

% img = imcomplement(img);
figure;
imshow(img);
title('Görsel Üzerinde Çizgi Çizin');
axis on; % Axis açılıyor
hold on;

% 2. Çizgi ve kesişim noktaları için boş yapı oluştur
lines = {}; % Çizgiler listesi
intersections = struct(); % Kesişim noktaları
markerSize = 10; % Nokta boyutu
threshold = 15; % Nokta yakınlık eşiği (pixel cinsinden)

% Çizgi numaralandırma ve kesişim noktası numaralandırma
lineCount = 0;
intersectionCount = 0;

while true
    % Zoom etkinleştiriliyor, kullanıcı yakınlaşabilir
    zoom on;
    disp('Zoom modunda, yakınlaşıp uzaklaşmak için fareyi kullanın. Çizgi seçimi için Enter tuşuna basın.');
    
    % Zoom'u kontrol etmek için bekleme
    pause; % Kullanıcı zoom'u tamamladığında Enter'a basar
    
    % Zoom işlemi kapatılıyor
    zoom off;
    
    % Kullanıcıdan çizgi çizmesini iste
    lineCount = lineCount + 1;
    disp(['Çizgi ', num2str(lineCount), ' için iki nokta seçin (İptal için Enter).']);
    [x, y, button] = ginput(2); % İki noktayı seçin
    
    % Eğer Enter'a basılırsa çıkış yapılır
    if isempty(x) || isempty(y) || length(x) ~= 2 || length(y) ~= 2
        disp('Çizim sonlandırıldı.');
        break;
    end
    
    % Koordinatların aynı olup olmadığını kontrol et (aynı koordinat çizgisi engelleme)
    if isequal([x(1), y(1)], [x(2), y(2)])
        disp('İki nokta aynı olamaz, lütfen farklı noktalar seçin.');
        continue;
    end

    % Önceden çizilmiş kesişim noktalarıyla yakınlık kontrolü
    if ~isFarFromExistingPoints([x, y], intersections, threshold)
        disp('Seçtiğiniz noktalar mevcut kesişim noktalarına çok yakın, lütfen farklı noktalar seçin.');
        continue;
    end

    % Yeni çizgiyi çiz ve kaydet
    newLine = plot([x(1), x(2)], [y(1), y(2)], 'LineWidth', 2, 'Color', 'b');
    lines{end+1} = [x(1), y(1); x(2), y(2)]; % Çizgi koordinatlarını kaydet
    
    % Önceden çizilmiş tüm çizgilerle kesişimi kontrol et
    for i = 1:lineCount-1
        % Eski çizgiyle yeni çizginin kesişimini bul
        x1 = lines{i}(1, 1); y1 = lines{i}(1, 2);
        x2 = lines{i}(2, 1); y2 = lines{i}(2, 2);
        
        % Çizgilerin doğrusal denklemleri: Ax + By = C formunda
        A1 = y1 - y2;
        B1 = x2 - x1;
        C1 = A1 * x1 + B1 * y1;
        
        A2 = y(1) - y(2);
        B2 = x(2) - x(1);
        C2 = A2 * x(1) + B2 * y(1);
        
        % Determinantı hesaplayarak paralel olup olmadıklarını kontrol et
        determinant = A1 * B2 - A2 * B1;
        
        if determinant ~= 0
            % Kesişim noktası bulunuyorsa
            x_intersect = (B2 * C1 - B1 * C2) / determinant;
            y_intersect = (A1 * C2 - A2 * C1) / determinant;
            
            % Kesişim noktası yalnızca çizgilerin segmentleri içinde olmalı
            if isPointOnSegment([x1, y1], [x2, y2], [x_intersect, y_intersect]) && ...
               isPointOnSegment([x(1), y(1)], [x(2), y(2)], [x_intersect, y_intersect])
               
                % 3. Kesişim noktası işaretleme ve numaralandırma
                intersectionCount = intersectionCount + 1;
                plot(x_intersect, y_intersect, 'go', 'MarkerSize', markerSize, 'LineWidth', 2);
                text(x_intersect + 5, y_intersect, num2str(intersectionCount), 'Color', 'yellow', 'FontSize', 36);
                
                % 4. Kesişim noktalarını workspace'de numaralandırarak kaydet
                intersections.(sprintf('intersection_%d', intersectionCount)) = [x_intersect, y_intersect];
                assignin('base', sprintf('intersection_%d', intersectionCount), [x_intersect, y_intersect]);
                
                % Çıktı olarak da göster
                disp(['Kesişim noktası ', num2str(intersectionCount), ':']);
                disp([x_intersect, y_intersect]);
            end
        end
    end
end

hold off;

% Kesişim noktalarını dosyaya yazma
[folderPath, ~, ~] = fileparts(imageFile); % Görüntünün bulunduğu klasör
outputFile = fullfile(folderPath, 'intersections.txt');
fid = fopen(outputFile, 'w');

fprintf(fid, 'Kesişim Noktaları (x, y):\n');
intersectionFields = fieldnames(intersections);
for i = 1:length(intersectionFields)
    intersectionData = intersections.(intersectionFields{i});
    fprintf(fid, '%s: %.2f, %.2f\n', intersectionFields{i}, intersectionData(1), intersectionData(2));
end

fclose(fid);
disp(['Kesişim noktaları "', outputFile, '" dosyasına kaydedildi.']);

% Yardımcı fonksiyon: Noktanın çizgi segmenti üzerinde olup olmadığını kontrol eder
function isOnSegment = isPointOnSegment(p1, p2, pt)
    % x ve y ekseninde sınır kontrolü
    isOnSegment = pt(1) >= min(p1(1), p2(1)) && pt(1) <= max(p1(1), p2(1)) && ...
                  pt(2) >= min(p1(2), p2(2)) && pt(2) <= max(p1(2), p2(2));
end

% Yardımcı fonksiyon: Seçilen noktaların mevcut kesişim noktalarından uzaklığını kontrol eder
function isFar = isFarFromExistingPoints(newPoints, intersections, threshold)
    isFar = true; % Varsayılan olarak uzakta olduğu varsayılıyor
    % Eğer intersections yapısı boşsa, direkt uzakta say
    if isempty(fieldnames(intersections))
        return; 
    end
    for i = 1:length(intersections)
        existingPoint = intersections.(sprintf('intersection_%d', i));
        % Eğer mevcut nokta yeni noktalara belirli bir mesafeden daha yakınsa
        if norm(newPoints(1, :) - existingPoint) < threshold || norm(newPoints(2, :) - existingPoint) < threshold
            isFar = false; % Çok yakın
            break;
        end
    end
end
