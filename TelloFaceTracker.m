function TelloFaceTracker()
    % --- INTERFEJS GRAFICZNY (GUI) ---
    fig = uifigure('Name', 'DJI Tello - Niezawodny Śledzący Dron', 'Position', [100 100 800 600], 'CloseRequestFcn', @onWindowClose);
    
    btnConnect = uibutton(fig, 'Position', [20, 520, 120, 40], 'Text', '1. Połącz', 'ButtonPushedFcn', @connectDrone);
    btnTakeoff = uibutton(fig, 'Position', [20, 460, 120, 40], 'Text', '2. Start', 'ButtonPushedFcn', @takeoffDrone);
    btnLand    = uibutton(fig, 'Position', [20, 400, 120, 40], 'Text', '3. Ląduj', 'ButtonPushedFcn', @landDrone);
    btnTrack   = uibutton(fig, 'Position', [20, 340, 120, 40], 'Text', '4. Kamera / Śledź', 'ButtonPushedFcn', @toggleTracking);
    
    ax = uiaxes(fig, 'Position', [160, 20, 620, 560]);
    title(ax, 'Podgląd na żywo');
    ax.XTick = []; ax.YTick = [];
    
    drone = [];
    cam = [];
    faceDetector = vision.CascadeObjectDetector('FrontalFaceCART');
    isTracking = false;
    hImg = [];
    
    scaleFactor = 0.5;        
    centerToleranceX = 30;    
    centerToleranceY = 30;    
    targetFaceSize = 65;      
    
    maxHeight = 2.0; 
    minHeight = 0.6; 
    
    function onWindowClose(src, ~)
        isTracking = false; 
        if ~isempty(drone) && isvalid(drone)
            try land(drone); catch; end
        end
        delete(src);
    end

    function connectDrone(~, ~)
        try
            drone = ryze();
            cam = camera(drone);
            uialert(fig, 'Połączono z dronem i kamerą pomyślnie!', 'Sukces', 'Icon', 'success');
        catch ME
            uialert(fig, ['Błąd połączenia: ' ME.message], 'Błąd', 'Icon', 'error');
        end
    end

    function takeoffDrone(~, ~)
        if ~isempty(drone), takeoff(drone); end
    end

    function landDrone(~, ~)
        isTracking = false; 
        if ~isempty(drone)
            try land(drone); catch; end
        end
    end

    function toggleTracking(~, ~)
        if isempty(drone) || isempty(cam)
            uialert(fig, 'Najpierw połącz się z dronem!', 'Błąd', 'Icon', 'warning');
            return;
        end
        
        isTracking = ~isTracking;
        if isTracking
            btnTrack.Text = 'Zatrzymaj Śledzenie';
            btnTrack.BackgroundColor = [1, 0.4, 0.4];
            runTrackingLoop();
        else
            btnTrack.Text = '4. Kamera / Śledź';
            btnTrack.BackgroundColor = [0.96, 0.96, 0.96];
        end
    end

    function runTrackingLoop()
        try
            initFrame = snapshot(cam);
            if isempty(initFrame)
                error('Kamera zwróciła pusty obraz podczas inicjalizacji.');
            end
            hImg = imshow(initFrame, 'Parent', ax);
        catch ME
            uialert(fig, ['Nie można pobrać pierwszej klatki: ' ME.message], 'Błąd Kamery', 'Icon', 'error');
            isTracking = false;
            btnTrack.Text = '4. Kamera / Śledź';
            btnTrack.BackgroundColor = [0.96, 0.96, 0.96];
            return;
        end
        
        frameCount = 0;
        batLevel = 0;
        currentHeight = 0;
        
        if ~isempty(drone)
            try batLevel = readBattery(drone); catch; end
        end
        
        while isTracking && isvalid(fig)
            frameCount = frameCount + 1;
            
            % 1. Bezpieczne pobieranie klatki ze sprawdzeniem, czy nie jest pusta
            try
                frame = snapshot(cam);
                if isempty(frame)
                    disp('[OSTRZEŻENIE] Kamera zwróciła pustą klatkę. Czekam na nową...');
                    pause(0.1);
                    continue;
                end
            catch camME
                disp(['[OSTRZEŻENIE] Zgubiono klatkę wideo: ' camME.message]);
                pause(0.1);
                continue;   
            end
            
            smallFrame = imresize(frame, scaleFactor);
            frameSize = size(smallFrame);
            frameCenter = [frameSize(2)/2, frameSize(1)/2]; 
            
            % 2. Telemetria
            try
                currentHeight = readHeight(drone);
                if mod(frameCount, 30) == 0
                    batLevel = readBattery(drone);
                end
            catch
            end
            
            % 3. OSD
            telemetryText = sprintf('Bateria: %d%%\nWysokosc: %.2f m', batLevel, currentHeight);
            textX = size(frame, 2) - 180; 
            frame = insertText(frame, [textX, 20], telemetryText, ...
                'FontSize', 16, 'BoxColor', 'black', 'TextColor', 'white', 'BoxOpacity', 0.6);
            
            % 4. Detekcja
            bboxes = step(faceDetector, smallFrame);
            maxErrorNorm = 0;
            moveIdx = 0; 
            
            if ~isempty(bboxes)
                [~, maxIdx] = max(bboxes(:,3));
                faceBoxSmall = bboxes(maxIdx, :);
                
                faceBox = faceBoxSmall / scaleFactor;
                frame = insertShape(frame, 'Rectangle', faceBox, 'Color', 'green', 'LineWidth', 3);
                
                faceCenter = [faceBoxSmall(1) + faceBoxSmall(3)/2, faceBoxSmall(2) + faceBoxSmall(4)/2];
                faceWidth = faceBoxSmall(3);
                
                errorX = faceCenter(1) - frameCenter(1);        
                errorY = frameCenter(2) - faceCenter(2);        
                sizeError = targetFaceSize - faceWidth;          
                
                normX = abs(errorX) / centerToleranceX;
                normY = abs(errorY) / centerToleranceY;
                normDist = abs(sizeError) / 15; 
                
                [maxErrorNorm, moveIdx] = max([normX, normY, normDist]);
            else
                frame = insertText(frame, [20, 20], 'PATROL: Szukam twarzy...', 'FontSize', 18, 'BoxColor', 'red');
            end
            
            % 5. Szybka aktualizacja ekranu
            hImg.CData = frame;
            drawnow limitrate;
            
            % 6. Zabezpieczony ruch z powiększonymi parametrami dla MATLAB API
            try
                if ~isempty(bboxes)
                    if maxErrorNorm > 1
                        if moveIdx == 1 % Yaw
                            if errorX > 0
                                turn(drone, deg2rad(-25)); % Zwiększono do 25 stopni
                            else
                                turn(drone, deg2rad(25));  
                            end
                        elseif moveIdx == 2 % Throttle
                            if errorY > 0 && currentHeight < maxHeight
                                moveup(drone, 0.2);        
                            elseif errorY < 0 && currentHeight > minHeight
                                movedown(drone, 0.2);
                            end
                        elseif moveIdx == 3 % Pitch
                            if sizeError > 0
                                moveforward(drone, 0.2);
                            else
                                moveback(drone, 0.2);
                            end
                        end
                        % Krótka pauza, aby dać dronowi czas na odzyskanie stabilności
                        pause(0.2);
                    end
                else
                    % Tryb patrolowania
                    turn(drone, deg2rad(25)); 
                    pause(0.2);
                end
                
                % Wyczyść zablokowane stare klatki powstałe w trakcie ruchu drona
                try snapshot(cam); catch; end 
                
            catch moveME
                disp(['[OSTRZEŻENIE] Zignorowano błąd komendy ruchu: ' moveME.message]);
            end
        end
    end
end