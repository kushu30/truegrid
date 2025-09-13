classdef main_simulator_app < matlab.apps.AppBase
    properties (Access = public)
        UIFigure             matlab.ui.Figure
        LoadMapButton        matlab.ui.control.Button
        CreateSimpleMapButton matlab.ui.control.Button
        FetchMapButton       matlab.ui.control.Button
        LatField             matlab.ui.control.NumericEditField
        LonField             matlab.ui.control.NumericEditField
        RadiusField          matlab.ui.control.NumericEditField
        NumPotholesSpinner   matlab.ui.control.Spinner
        NumBarricadesSpinner matlab.ui.control.Spinner
        BaseSpeedEdit        matlab.ui.control.NumericEditField
        FollowCameraCheck    matlab.ui.control.CheckBox
        StartButton          matlab.ui.control.Button
        PauseResumeButton    matlab.ui.control.Button
        StopButton           matlab.ui.control.Button
        UIAxes               matlab.ui.control.UIAxes
        LogTextArea          matlab.ui.control.TextArea
    end

    properties (Access = private)
        scenario
        car
        carPathWaypoints
        obstaclePositions
        obstacleTypes
        vehiclePlotHandle
        simTimer
        isPaused = false
        frameCount = 0
        restartAttempts = 0
        maxRestartAttempts = 3
        totalDistance = 0
        lastPosition
        lastPathIndex = 1
        
        % Enhanced properties for obstacle avoidance
        maxSpeed_ms = 15
        minSpeed_ms = 3
        obstacleDetectionRange = 35
        curvatureWeight = 3
        obstacleWeight = 8
        lastSpeedReason = 'Ready'
    end

    methods (Access = private)
        function startup(app)
            app.NumPotholesSpinner.Value = 2;
            app.NumBarricadesSpinner.Value = 1;
            app.BaseSpeedEdit.Value = 15;
            app.FollowCameraCheck.Value = false;
            app.LatField.Value = 12.9716;
            app.LonField.Value = 77.5946;
            app.RadiusField.Value = 200;
            app.LogTextArea.Value = "Ready. Click 'Create Simple Map' for easy testing.";
        end

        function CreateSimpleMapButtonPushed(app,~,~)
            app.log("Creating simple test map...");
            app.createSimpleTestMap();
        end

        function createSimpleTestMap(app)
            app.scenario = drivingScenario;
            
            waypoints = [
                0,    0;
                100,  0;
                150,  10;
                200,  30;
                250,  40;
                400,  40;
                500,  40
            ];
            
            road(app.scenario, waypoints);
            
            app.carPathWaypoints = waypoints;
            app.car = vehicle(app.scenario, 'ClassID', 1, 'Position', [waypoints(1,1:2) 0]);
            app.lastPosition = app.car.Position;
            app.setupPlot();
            app.lastPathIndex = 1;
            app.log("Simple test map created successfully!");
        end

        function LoadMapButtonPushed(app,~,~)
            [file,path] = uigetfile('*.osm','Select OSM map file');
            if isequal(file,0), return; end
            osmFile = fullfile(path,file);
            app.log("Loading local OSM: " + osmFile);
            app.loadOSM(osmFile);
        end

        function FetchMapButtonPushed(app,~,~)
            lat = app.LatField.Value; lon = app.LonField.Value; r = app.RadiusField.Value;
            url = sprintf(['https://overpass-api.de/api/interpreter?data=[out:xml];', ...
                           'way(around:%d,%f,%f)["highway"];(._;>;);out;'], r, lat, lon);
            tmpFile = fullfile(tempdir,'map.osm');
            try
                websave(tmpFile,url);
                app.log(sprintf("Fetched OSM from Overpass API at (%.5f,%.5f) radius %dm",lat,lon,r));
                app.loadOSM(tmpFile);
            catch ME
                app.log("Failed to fetch OSM: " + ME.message);
            end
        end

        function loadOSM(app,osmFile)
            xDoc = xmlread(osmFile);
            nodeList = xDoc.getElementsByTagName('node');
            nodes = containers.Map('KeyType','char','ValueType','any');
            for i=0:nodeList.getLength-1
                n = nodeList.item(i);
                id = char(n.getAttribute('id'));
                lat = str2double(n.getAttribute('lat'));
                lon = str2double(n.getAttribute('lon'));
                nodes(id) = [lat, lon];
            end
            wayList = xDoc.getElementsByTagName('way');
            app.scenario = drivingScenario;
            for i=0:wayList.getLength-1
                w = wayList.item(i);
                tags = w.getElementsByTagName('tag');
                isHighway = false;
                for j=0:tags.getLength-1
                    k = tags.item(j);
                    if strcmp(char(k.getAttribute('k')),'highway')
                        isHighway = true; break;
                    end
                end
                if ~isHighway, continue; end
                nds = w.getElementsByTagName('nd');
                coords = [];
                for j=0:nds.getLength-1
                    ref = char(nds.item(j).getAttribute('ref'));
                    if isKey(nodes,ref)
                        coords = [coords; nodes(ref)];
                    end
                end
                if size(coords,1)>=2
                    [x,y] = app.deg2utm(coords(:,1),coords(:,2));
                    road(app.scenario,[x y]);
                end
            end
            rs = app.scenario.RoadSegments;
            if numel(rs)>=1
                app.carPathWaypoints = rs(1).RoadCenters;
                app.car = vehicle(app.scenario,'ClassID',1,'Position',[app.carPathWaypoints(1,1:2) 0]);
                app.lastPosition = app.car.Position;
                app.setupPlot();
            else
                error('No road segments parsed from OSM.');
            end
            app.lastPathIndex = 1;
        end

        function setupPlot(app)
            cla(app.UIAxes);
            hold(app.UIAxes,'on');
            axis(app.UIAxes,'equal'); grid(app.UIAxes,'on');
            xlabel(app.UIAxes,'X (m)'); ylabel(app.UIAxes,'Y (m)');
            if ~isempty(app.scenario)
                app.plotDetailedRoads(app.scenario);
            end
            plot(app.UIAxes, app.carPathWaypoints(:,1), app.carPathWaypoints(:,2), 'r--', 'LineWidth', 2);
            cp = app.car.Position;
            app.vehiclePlotHandle = plot(app.UIAxes, cp(1), cp(2), 'bo','MarkerSize',12,'MarkerFaceColor','b');
            drawnow;
            app.log("Plot setup complete.");
        end

        function StartButtonPushed(app,~,~)
            if isempty(app.scenario)
                app.log("Create a map first using 'Create Simple Map' or load an OSM file.");
                return;
            end
            
            app.maxSpeed_ms = app.BaseSpeedEdit.Value;
            app.minSpeed_ms = max(3, app.maxSpeed_ms * 0.2);
            
            np = round(app.NumPotholesSpinner.Value);
            nb = round(app.NumBarricadesSpinner.Value);
            [app.obstaclePositions, app.obstacleTypes] = app.createObstacles(np, nb);
            app.plotObstacles();
            
            baseSpeed = app.BaseSpeedEdit.Value;
            path(app.car, app.carPathWaypoints, baseSpeed);
            app.frameCount = 0; app.totalDistance = 0; 
            app.lastPosition = app.car.Position;
            app.lastPathIndex = 1;
            app.restartAttempts = 0;
            app.lastSpeedReason = 'Starting simulation';
            
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer); delete(app.simTimer);
            end
            app.simTimer = timer('ExecutionMode','fixedSpacing','Period',0.05,'TimerFcn',@(~,~)app.timerStep());
            start(app.simTimer);
            app.isPaused = false;
            app.log('Simulation started with enhanced obstacle avoidance.');
        end

        function timerStep(app)
            try
                if app.isPaused, return; end

                try
                    advance(app.scenario);
                catch advErr
                    msg = string(advErr.message);
                    if contains(lower(msg),'not running') || contains(lower(msg),'simulation is not running')
                        app.log("Scenario stopped unexpectedly: " + msg);
                        if app.restartAttempts < app.maxRestartAttempts
                            app.restartAttempts = app.restartAttempts + 1;
                            try
                                restart(app.scenario);
                                app.log("Restarted scenario (attempt " + app.restartAttempts + "). Reapplying path.");
                                poses = actorPoses(app.scenario);
                                if ~isempty(poses)
                                    carPos = poses(1).Position;
                                else
                                    carPos = app.car.Position;
                                end
                                [~, ~, idx] = app.snap2path(app.carPathWaypoints, carPos');
                                idx = max(1, min(idx, size(app.carPathWaypoints,1)));
                                remainingWindow = 150;
                                remaining = app.carPathWaypoints(idx:min(idx+remainingWindow,end), :);
                                if size(remaining,1) >= 2
                                    try
                                        path(app.car, remaining, app.BaseSpeedEdit.Value);
                                    catch pErr
                                        app.log("Path reapply failed: " + pErr.message);
                                    end
                                else
                                    app.log("No sufficient remaining waypoints after restart.");
                                end
                                return;
                            catch restartErr
                                app.log("Restart attempt failed: " + restartErr.message);
                                app.stopAndCleanup();
                                return;
                            end
                        else
                            app.log("Max restart attempts reached. Stopping simulation.");
                            app.stopAndCleanup();
                            return;
                        end
                    else
                        rethrow(advErr);
                    end
                end

                app.frameCount = app.frameCount + 1;
                poses = actorPoses(app.scenario);
                if isempty(poses), return; end
                carPose = poses(1);
                carPos = carPose.Position;
                carVel = carPose.Velocity;

                dThis = norm(carPos - app.lastPosition);
                app.totalDistance = app.totalDistance + dThis;
                app.lastPosition = carPos;

                [~, ~, idx] = app.snap2path(app.carPathWaypoints, carPos');
                maxStep = 1;
                if idx < app.lastPathIndex
                    idx = app.lastPathIndex;
                elseif idx > app.lastPathIndex + maxStep
                    idx = app.lastPathIndex + maxStep;
                end

                curvature = app.calculatePathCurvature(app.carPathWaypoints, idx);
                curvatureFactor = 1/(1 + app.curvatureWeight * curvature);
                
                [obsDist, obsType, ~] = app.findNearestObstacle(carPos, app.obstaclePositions, app.obstacleTypes);
                obstacleFactor = app.calculateObstacleFactor(obsDist, obsType, app.obstacleDetectionRange);
                
                combinedFactor = min(curvatureFactor, obstacleFactor);
                targetSpeed = app.minSpeed_ms + (app.maxSpeed_ms - app.minSpeed_ms) * combinedFactor;
                targetSpeed = max(app.minSpeed_ms, min(app.maxSpeed_ms, targetSpeed));

                app.lastSpeedReason = app.determineSpeedReason(curvature, obsDist, obsType, app.obstacleDetectionRange);

                updateThreshold = 4;
                periodicFrames = 25;
                advancedSinceLastUpdate = idx - app.lastPathIndex;
                needUpdate = (advancedSinceLastUpdate >= updateThreshold) || (mod(app.frameCount, periodicFrames) == 0);

                remainingWindow = 120;
                startIdx = idx;
                endIdx = min(idx + remainingWindow, size(app.carPathWaypoints,1));
                remaining = app.carPathWaypoints(startIdx:endIdx, :);
                
                if obsDist < app.obstacleDetectionRange && ~isempty(remaining)
                    remaining = app.applyObstacleAvoidance(remaining, carPos, obsDist, obsType);
                    
                    if startIdx + size(remaining,1) - 1 <= size(app.carPathWaypoints,1)
                        app.carPathWaypoints(startIdx:(startIdx + size(remaining,1) - 1), :) = remaining;
                    else
                        app.carPathWaypoints(startIdx:end, :) = remaining(1:(size(app.carPathWaypoints,1) - startIdx + 1), :);
                    end
                end

                if needUpdate && size(remaining,1) >= 2
                    try
                        path(app.car, app.carPathWaypoints(startIdx:end), targetSpeed);
                        app.lastPathIndex = idx;
                    catch pErr
                        app.log("path() error: " + pErr.message);
                    end
                end

                if isvalid(app.vehiclePlotHandle)
                    set(app.vehiclePlotHandle,'XData',carPos(1),'YData',carPos(2));
                else
                    app.vehiclePlotHandle = plot(app.UIAxes,carPos(1),carPos(2),'bo','MarkerSize',12,'MarkerFaceColor','b');
                end

                if app.FollowCameraCheck.Value
                    zl = 70;
                    xlim(app.UIAxes,[carPos(1)-zl carPos(1)+zl]);
                    ylim(app.UIAxes,[carPos(2)-zl carPos(2)+zl]);
                end

                actualSpeed_ms = norm(carVel);
                actualSpeed_kmh = actualSpeed_ms * 3.6;
                targetSpeed_kmh = targetSpeed * 3.6;
                avgSpeed_kmh = (app.totalDistance / max(app.scenario.SimulationTime, 0.1)) * 3.6;
                
                title(app.UIAxes,sprintf('T=%.1fs | Target: %.0f km/h | Actual: %.0f km/h | %s | Dist: %.0fm | Avg: %.0f km/h', ...
                    app.scenario.SimulationTime, targetSpeed_kmh, actualSpeed_kmh, app.lastSpeedReason, app.totalDistance, avgSpeed_kmh));

                if mod(app.frameCount, 30) == 0
                    app.log(sprintf('Frame %d: Time=%.1fs, Speed=%.0f/%.0f km/h, %s, Obstacle dist=%.1fm', ...
                        app.frameCount, app.scenario.SimulationTime, actualSpeed_kmh, targetSpeed_kmh, ...
                        app.lastSpeedReason, obsDist));
                end

            catch ME
                app.log("Timer error: " + ME.message);
                app.stopAndCleanup();
            end
        end
        
        function modifiedPath = applyObstacleAvoidance(app, originalPath, carPos, obstacleDistance, obstacleType)
            modifiedPath = originalPath;
            
            if obstacleDistance > 15
                return;
            end
            
            [~, ~, obsIdx] = app.findNearestObstacle(carPos, app.obstaclePositions, app.obstacleTypes);
            if obsIdx == 0
                return;
            end
            
            obstaclePos = app.obstaclePositions(obsIdx, :);
            
            avoidanceDistance = 2.5;
            if obstacleType == 3
                avoidanceDistance = 3.5;
            end
            
            for i = 1:size(modifiedPath, 1)
                distToObs = norm(modifiedPath(i, 1:2) - obstaclePos(1:2));
                if distToObs < avoidanceDistance * 2
                    if size(modifiedPath, 1) == 1
                        continue;
                    end
                    
                    if i == 1
                        pathDir = modifiedPath(i+1, 1:2) - modifiedPath(i, 1:2);
                    elseif i == size(modifiedPath, 1)
                        pathDir = modifiedPath(i, 1:2) - modifiedPath(i-1, 1:2);
                    else
                        pathDir = modifiedPath(i+1, 1:2) - modifiedPath(i-1, 1:2);
                    end
                    
                    if norm(pathDir) > 0
                        pathDir = pathDir / norm(pathDir);
                        perpDir = [-pathDir(2), pathDir(1)];
                        
                        toObstacle = obstaclePos(1:2) - modifiedPath(i, 1:2);
                        if dot(perpDir, toObstacle) > 0
                            perpDir = -perpDir;
                        end
                        
                        avoidanceFactor = max(0, 1 - distToObs / (avoidanceDistance * 2));
                        offset = perpDir * avoidanceDistance * avoidanceFactor;
                        modifiedPath(i, 1:2) = modifiedPath(i, 1:2) + offset;
                    end
                end
            end
        end

        function PauseResumeButtonPushed(app,~,~)
            if isempty(app.simTimer) || ~isvalid(app.simTimer), return; end
            app.isPaused = ~app.isPaused;
            app.log(app.isPaused*"Paused." + ~app.isPaused*"Resumed.");
        end

        function StopButtonPushed(app,~,~)
            app.stopAndCleanup();
        end
        
        function stopAndCleanup(app)
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer); delete(app.simTimer); app.simTimer = [];
            end
            
            if app.frameCount > 0 && app.scenario.SimulationTime > 0
                avgSpeed_kmh = (app.totalDistance / app.scenario.SimulationTime) * 3.6;
                app.log(sprintf('=== SIMULATION COMPLETE ==='));
                app.log(sprintf('Frames: %d | Time: %.1fs | Distance: %.0fm | Avg Speed: %.0f km/h', ...
                    app.frameCount, app.scenario.SimulationTime, app.totalDistance, avgSpeed_kmh));
            else
                app.log("Simulation stopped.");
            end
        end

        function log(app,msg)
            existing = app.LogTextArea.Value;
            if ischar(existing), existing = string(existing); end
            app.LogTextArea.Value = [existing; string(sprintf('[%s] %s', datestr(now,'HH:MM:SS'), msg))];
            drawnow;
        end

        function plotObstacles(app)
            cla(app.UIAxes);
            if ~isempty(app.scenario), app.plotDetailedRoads(app.scenario); end
            plot(app.UIAxes, app.carPathWaypoints(:,1), app.carPathWaypoints(:,2), 'r--','LineWidth',2);
            for i=1:size(app.obstaclePositions,1)
                pos = app.obstaclePositions(i,:); typ = app.obstacleTypes(i);
                if typ==2
                    plot(app.UIAxes,pos(1),pos(2),'kx','MarkerSize',15,'LineWidth',3);
                elseif typ==3
                    plot(app.UIAxes,pos(1),pos(2),'^','MarkerSize',15,'MarkerFaceColor',[1 0.8 0],'MarkerEdgeColor','black','LineWidth',2);
                end
            end
            cp = app.car.Position;
            app.vehiclePlotHandle = plot(app.UIAxes,cp(1),cp(2),'bo','MarkerSize',12,'MarkerFaceColor','b');
            drawnow;
        end

        function [positions, types] = createObstacles(app,npotholes,nbarricades)
            positions = []; types = [];
            L = size(app.carPathWaypoints,1); 
            rng('shuffle');
            
            for i=1:npotholes
                idx = randi([round(L*0.2), round(L*0.9)]);
                pos = app.carPathWaypoints(idx,:); 
                if size(pos,2)==2, pos=[pos 0]; end
                pos(1:2) = pos(1:2) + (rand(1,2) - 0.5) * 2;
                positions = [positions; pos]; 
                types = [types; 2];
            end
            
            for i=1:nbarricades
                idx = randi([round(L*0.3), round(L*0.95)]);
                pos = app.carPathWaypoints(idx,:); 
                if size(pos,2)==2, pos=[pos 0]; end
                lateral = (rand-0.5)*4;
                pos2 = pos+[lateral lateral*0.2 0];
                positions = [positions; pos2]; 
                types = [types; 3];
            end
        end

        function plotDetailedRoads(app, scenario)
            ax = app.UIAxes; hold(ax,'on');
            roadColor = [0.4 0.4 0.4];
            centerLineColor = [1 1 0];
            
            for i=1:length(scenario.RoadSegments)
                roadCenters = scenario.RoadSegments(i).RoadCenters;
                plot(ax,roadCenters(:,1),roadCenters(:,2),'-','Color',roadColor,'LineWidth',6);
                plot(ax,roadCenters(:,1),roadCenters(:,2),'-','Color',centerLineColor,'LineWidth',1.5);
            end
        end

        function [x,y] = deg2utm(~,lat,lon)
            R=6371000; lat0=mean(lat); lon0=mean(lon);
            x=R*(deg2rad(lon-lon0)).*cosd(lat0); y=R*deg2rad(lat-lat0);
        end

        function [distance, obstacleType, idx] = findNearestObstacle(~, carPosition, obstaclePositions, obstacleTypes)
            if isempty(obstaclePositions), distance=inf; obstacleType=0; idx=0; return; end
            dvec = sqrt(sum((obstaclePositions(:,1:2)-carPosition(1:2)).^2,2));
            [distance,idx] = min(dvec); obstacleType = obstacleTypes(idx);
        end

        function factor = calculateObstacleFactor(~, dist, type, range)
            if dist > range, factor = 1.0; return; end
            
            switch type
                case 2
                    severityMultiplier = 0.6;
                case 3  
                    severityMultiplier = 0.3;
                otherwise
                    severityMultiplier = 0.5;
            end
            
            proximityFactor = dist / range;
            factor = severityMultiplier + (1 - severityMultiplier) * proximityFactor;
            factor = max(0.1, min(1.0, factor));
        end

        function curvature = calculatePathCurvature(~,pathWaypoints,idx)
            lookahead = min(idx+5,size(pathWaypoints,1));
            future = min(idx+10,size(pathWaypoints,1));
            if idx>=size(pathWaypoints,1)-10, curvature=0; return; end
            
            v1 = pathWaypoints(lookahead,:) - pathWaypoints(idx,:);
            v2 = pathWaypoints(future,:) - pathWaypoints(lookahead,:);
            if norm(v1)<eps||norm(v2)<eps, curvature=0; return; end
            
            cosA = dot(v1,v2)/(norm(v1)*norm(v2)); 
            cosA = max(-1,min(1,cosA));
            angle = acosd(cosA);
            curvature = abs(angle)/90;
        end
        
        function reason = determineSpeedReason(~, curvature, obstacleDistance, obstacleType, detectionRange)
            if obstacleDistance <= detectionRange
                switch obstacleType
                    case 2
                        reason = sprintf('Pothole ahead (%.1fm)', obstacleDistance);
                    case 3
                        reason = sprintf('Barricade ahead (%.1fm)', obstacleDistance);
                    otherwise
                        reason = sprintf('Obstacle ahead (%.1fm)', obstacleDistance);
                end
            elseif curvature > 0.3
                reason = sprintf('Sharp curve (%.2f)', curvature);
            elseif curvature > 0.1
                reason = sprintf('Moderate curve (%.2f)', curvature);
            else
                reason = 'Clear path - cruising';
            end
        end

        function [closestPoint,d,idx] = snap2path(~,pathWaypoints,point)
            if size(point,1) > size(point,2)
                point = point';
            end
            if size(pathWaypoints,2) > 2
                pathWaypoints = pathWaypoints(:,1:2);
            end
            
            if size(point,2) == 1 && size(pathWaypoints,2) == 2
                error('Point and path dimensions incompatible');
            elseif size(point,2) == 2 && size(pathWaypoints,2) == 2
                dists = sqrt(sum((pathWaypoints - point).^2, 2));
            else
                minDim = min(size(point,2), size(pathWaypoints,2));
                dists = sqrt(sum((pathWaypoints(:,1:minDim) - point(1:minDim)).^2, 2));
            end
            
            [d,idx] = min(dists); 
            closestPoint = pathWaypoints(idx,:);
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure=uifigure('Name','Enhanced Vehicle Simulator with Obstacle Avoidance','Position',[100 100 1300 750]);
            
            app.LoadMapButton=uibutton(app.UIFigure,'push','Position',[20 700 80 30],'Text','Load OSM','ButtonPushedFcn',@(btn,event)app.LoadMapButtonPushed(btn,event));
            app.CreateSimpleMapButton=uibutton(app.UIFigure,'push','Position',[110 700 120 30],'Text','Create Simple Map','ButtonPushedFcn',@(btn,event)app.CreateSimpleMapButtonPushed(btn,event));
            
            uilabel(app.UIFigure,'Position',[240 705 30 20],'Text','Lat'); 
            app.LatField=uieditfield(app.UIFigure,'numeric','Position',[270 705 70 22]);
            uilabel(app.UIFigure,'Position',[350 705 30 20],'Text','Lon'); 
            app.LonField=uieditfield(app.UIFigure,'numeric','Position',[380 705 70 22]);
            uilabel(app.UIFigure,'Position',[460 705 50 20],'Text','Radius'); 
            app.RadiusField=uieditfield(app.UIFigure,'numeric','Position',[510 705 50 22]);
            app.FetchMapButton=uibutton(app.UIFigure,'push','Position',[570 700 120 30],'Text','Fetch from OSM','ButtonPushedFcn',@(btn,event)app.FetchMapButtonPushed(btn,event));
            
            uilabel(app.UIFigure,'Position',[20 660 120 20],'Text','Num Potholes'); 
            app.NumPotholesSpinner=uispinner(app.UIFigure,'Position',[140 660 80 22],'Limits',[0 50],'Value',2);
            uilabel(app.UIFigure,'Position',[240 660 120 20],'Text','Num Barricades'); 
            app.NumBarricadesSpinner=uispinner(app.UIFigure,'Position',[360 660 80 22],'Limits',[0 50],'Value',1);
            uilabel(app.UIFigure,'Position',[20 620 100 20],'Text','Base Speed (m/s)'); 
            app.BaseSpeedEdit=uieditfield(app.UIFigure,'numeric','Position',[140 620 80 22],'Value',15);
            app.FollowCameraCheck=uicheckbox(app.UIFigure,'Position',[260 620 120 22],'Text','Follow Camera','Value',false);
            
            app.StartButton=uibutton(app.UIFigure,'push','Position',[20 580 80 30],'Text','Start','ButtonPushedFcn',@(btn,event)app.StartButtonPushed(btn,event));
            app.PauseResumeButton=uibutton(app.UIFigure,'push','Position',[110 580 100 30],'Text','Pause/Resume','ButtonPushedFcn',@(btn,event)app.PauseResumeButtonPushed(btn,event));
            app.StopButton=uibutton(app.UIFigure,'push','Position',[220 580 80 30],'Text','Stop','ButtonPushedFcn',@(btn,event)app.StopButtonPushed(btn,event));
            
            app.UIAxes=uiaxes(app.UIFigure,'Position',[20 20 950 540]); 
            title(app.UIAxes,'Enhanced Vehicle Simulation'); 
            xlabel(app.UIAxes,'X'); ylabel(app.UIAxes,'Y');
            app.LogTextArea=uitextarea(app.UIFigure,'Position',[1000 20 280 710],'Editable','off');
        end
    end

    methods (Access = public)
        function app=main_simulator_app
            createComponents(app); startup(app);
        end
        function delete(app)
            if ~isempty(app.simTimer)&&isvalid(app.simTimer), stop(app.simTimer); delete(app.simTimer); end
            delete(app.UIFigure);
        end
    end
end