classdef truegrid_sim < matlab.apps.AppBase
    properties (Access = public)
        UIFigure
        LoadMapButton
        CreateSimpleMapButton
        FetchMapButton
        LatField
        LonField
        RadiusField
        NumPotholesSpinner
        NumBarricadesSpinner
        BaseSpeedEdit
        FollowCameraCheck
        StartButton
        PauseResumeButton
        StopButton
        UIAxes
        LogTextArea
    end

    properties (Access = private)
        scenario
        car
        cars
        carPathWaypoints
        obstaclePositions
        obstacleTypes
        obstacleProps
        vehiclePlotHandle
        simTimer
        isPaused = false
        frameCount = 0
        restartAttempts = 0
        maxRestartAttempts = 3
        totalDistance = 0
        lastPosition
        lastPathIndex = 1
        maxSpeed_ms = 15
        minSpeed_ms = 3
        obstacleDetectionRange = 35
        curvatureWeight = 3
        obstacleWeight = 8
        lastSpeedReason = 'Ready'
        currentSpeed = 15
    end

    methods (Access = private)
        function exportFrame(app)
            poses = [];
            try
                poses = actorPoses(app.scenario);
            catch
                poses = [];
            end

            if isempty(poses)
                carsOut = [];
            else
                carsOut = cell(1, numel(poses));
                for i = 1:numel(poses)
                    p = poses(i);
                    carsOut{i} = struct( ...
                        'id', i, ...
                        'x', double(p.Position(1)), ...
                        'y', double(p.Position(2)), ...
                        'z', double(p.Position(3)), ...
                        'vx', double(p.Velocity(1)), ...
                        'vy', double(p.Velocity(2)), ...
                        'vz', double(p.Velocity(3)), ...
                        'speed_m_s', double(norm(p.Velocity)), ...
                        'speed_km_h', double(norm(p.Velocity)) * 3.6 ...
                    );
                end
                carsOut = [carsOut{:}];
            end

            if isempty(app.obstaclePositions)
                obstaclesOut = [];
            else
                nObs = size(app.obstaclePositions, 1);
                tempObs = cell(1, nObs);
                for i = 1:nObs
                    pos = app.obstaclePositions(i, :);
                    prop = struct('severity', 1, 'width', 1.0, 'active', true);
                    if ~isempty(app.obstacleProps) && length(app.obstacleProps) >= i
                        try
                            prop.severity = app.obstacleProps(i).severity;
                            prop.width = app.obstacleProps(i).width;
                            prop.active = app.obstacleProps(i).active;
                        catch
                            prop.severity = 1;
                            prop.width = 1.0;
                            prop.active = true;
                        end
                    end
                    tempObs{i} = struct( ...
                        'id', i, ...
                        'x', double(pos(1)), ...
                        'y', double(pos(2)), ...
                        'z', double(pos(3)), ...
                        'type', double(app.obstacleTypes(i)), ...
                        'severity', double(prop.severity), ...
                        'width', double(prop.width), ...
                        'active', logical(prop.active) ...
                    );
                end
                obstaclesOut = [tempObs{:}];
            end

            simTime = 0;
            try
                simTime = app.scenario.SimulationTime;
            catch
                simTime = 0;
            end
            avgSp = double((app.totalDistance / max(0.1, simTime)) * 3.6);
            telemetry = struct( ...
                'time', double(simTime), ...
                'distance', double(app.totalDistance), ...
                'avgSpeed_km_h', avgSp, ...
                'lastSpeedReason', app.lastSpeedReason, ...
                'currentSpeed_ms', double(app.currentSpeed), ...
                'currentSpeed_kmh', double(app.currentSpeed * 3.6), ...
                'avgSpeed', avgSp ...
            );

            pathPoints = [];
            if ~isempty(app.carPathWaypoints)
                pathPoints = arrayfun(@(i) struct('x', app.carPathWaypoints(i, 1), 'y', app.carPathWaypoints(i, 2)), 1:size(app.carPathWaypoints, 1));
            end

            frame = struct('cars', {carsOut}, 'obstacles', {obstaclesOut}, 'telemetry', telemetry, 'path', {pathPoints});
            jsonStr = jsonencode(frame);

            outputPath = 'C:\Users\abhay\Downloads\sih\web';
            finalFile = fullfile(outputPath, 'simframe.json');

            tmpFile = [finalFile, '.tmp'];
            fid = fopen(tmpFile, 'w');
            if fid == -1
                app.log("ERROR: Cannot write to JSON file. Check permissions and path.");
                return;
            end
            fwrite(fid, jsonStr, 'char');
            fclose(fid);
            try
                movefile(tmpFile, finalFile, 'f');
            catch
                try
                    copyfile(tmpFile, finalFile);
                    delete(tmpFile);
                catch
                end
            end
        end

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

        function CreateSimpleMapButtonPushed(app, ~, ~)
            app.createSimpleTestMap();
        end

        function createSimpleTestMap(app)
            app.scenario = drivingScenario;
            waypoints = [0 0; 100 0; 150 10; 200 30; 250 40; 400 40; 500 40];
            road(app.scenario, waypoints);
            app.carPathWaypoints = waypoints;
            app.car = vehicle(app.scenario, 'ClassID', 1, 'Position', [waypoints(1, 1:2) 0]);
            app.cars = [app.car, vehicle(app.scenario, 'ClassID', 1, 'Position', [5 10 0]), vehicle(app.scenario, 'ClassID', 1, 'Position', [-5 20 0])];
            app.lastPosition = app.car.Position;
            app.setupPlot();
            app.lastPathIndex = 1;
        end

        function LoadMapButtonPushed(app, ~, ~)
            [file, path] = uigetfile('*.osm', 'Select OSM map file');
            if isequal(file, 0)
                return;
            end
            osmFile = fullfile(path, file);
            app.loadOSM(osmFile);
        end

        function FetchMapButtonPushed(app, ~, ~)
            lat = app.LatField.Value;
            lon = app.LonField.Value;
            r = app.RadiusField.Value;
            url = sprintf(['https://overpass-api.de/api/interpreter?data=[out:xml];way(around:%d,%f,%f)["highway"];(._;>;);out;'], r, lat, lon);
            tmpFile = fullfile(tempdir, 'map.osm');
            try
                websave(tmpFile, url);
                app.loadOSM(tmpFile);
            catch
            end
        end

        function loadOSM(app, osmFile)
            xDoc = xmlread(osmFile);
            nodeList = xDoc.getElementsByTagName('node');
            nodes = containers.Map('KeyType', 'char', 'ValueType', 'any');
            for i = 0:nodeList.getLength-1
                n = nodeList.item(i);
                id = char(n.getAttribute('id'));
                lat = str2double(n.getAttribute('lat'));
                lon = str2double(n.getAttribute('lon'));
                nodes(id) = [lat, lon];
            end
            wayList = xDoc.getElementsByTagName('way');
            app.scenario = drivingScenario;
            for i = 0:wayList.getLength-1
                w = wayList.item(i);
                tags = w.getElementsByTagName('tag');
                isHighway = false;
                for j = 0:tags.getLength-1
                    k = tags.item(j);
                    if strcmp(char(k.getAttribute('k')), 'highway')
                        isHighway = true;
                        break;
                    end
                end
                if ~isHighway
                    continue;
                end
                nds = w.getElementsByTagName('nd');
                coords = [];
                for j = 0:nds.getLength-1
                    ref = char(nds.item(j).getAttribute('ref'));
                    if isKey(nodes, ref)
                        coords = [coords; nodes(ref)];
                    end
                end
                if size(coords, 1) >= 2
                    [x, y] = app.deg2utm(coords(:, 1), coords(:, 2));
                    road(app.scenario, [x y]);
                end
            end
            rs = app.scenario.RoadSegments;
            if numel(rs) >= 1
                app.carPathWaypoints = rs(1).RoadCenters;
                app.car = vehicle(app.scenario, 'ClassID', 1, 'Position', [app.carPathWaypoints(1, 1:2) 0]);
                app.cars = [app.car];
                app.lastPosition = app.car.Position;
                app.setupPlot();
            else
                error('No road segments parsed from OSM.');
            end
            app.lastPathIndex = 1;
        end

        function setupPlot(app)
            cla(app.UIAxes);
            hold(app.UIAxes, 'on');
            axis(app.UIAxes, 'equal');
            grid(app.UIAxes, 'on');
            xlabel(app.UIAxes, 'X (m)');
            ylabel(app.UIAxes, 'Y (m)');
            if ~isempty(app.scenario)
                app.plotDetailedRoads(app.scenario);
            end
            plot(app.UIAxes, app.carPathWaypoints(:, 1), app.carPathWaypoints(:, 2), 'r--', 'LineWidth', 2);
            cp = app.car.Position;
            app.vehiclePlotHandle = plot(app.UIAxes, cp(1), cp(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
            drawnow;
        end

        function StartButtonPushed(app, ~, ~)
            if isempty(app.scenario)
                return;
            end
            app.maxSpeed_ms = app.BaseSpeedEdit.Value;
            app.minSpeed_ms = max(3, app.maxSpeed_ms * 0.2);
            app.currentSpeed = app.maxSpeed_ms;
            np = round(app.NumPotholesSpinner.Value);
            nb = round(app.NumBarricadesSpinner.Value);
            [app.obstaclePositions, app.obstacleTypes, app.obstacleProps] = app.createObstacles(np, nb);
            app.plotObstacles();
            path(app.car, app.carPathWaypoints, app.currentSpeed);
            app.frameCount = 0;
            app.totalDistance = 0;
            app.lastPosition = app.car.Position;
            app.lastPathIndex = 1;
            app.restartAttempts = 0;
            app.lastSpeedReason = 'Starting simulation';
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer);
                delete(app.simTimer);
            end
            app.simTimer = timer('ExecutionMode', 'fixedSpacing', 'Period', 0.05, 'TimerFcn', @(~, ~)app.timerStep());
            start(app.simTimer);
            app.isPaused = false;
            app.log("Simulation started");
        end

        function timerStep(app)
            try
                if app.isPaused
                    return;
                end
                poses = actorPoses(app.scenario);
                if isempty(poses)
                    return;
                end
                carPose = poses(1);
                carPos = carPose.Position;
                [~, ~, idx] = app.snap2path(app.carPathWaypoints, carPos');
                curvature = app.calculatePathCurvature(app.carPathWaypoints, idx);
                [obsDist, obsType, obsIdx] = app.findNearestObstacle(carPos, app.obstaclePositions, app.obstacleTypes);
                newSpeed = app.calculateDynamicSpeed(curvature, obsDist, obsType);
                if abs(newSpeed - app.currentSpeed) > 0.1
                    app.currentSpeed = newSpeed;
                    reason = app.determineSpeedReason(curvature, obsDist, obsType, app.obstacleDetectionRange);
                    app.lastSpeedReason = reason;
                    app.log(sprintf("Speed updated: %.1f m/s (%.1f km/h) - %s", app.currentSpeed, app.currentSpeed * 3.6, reason));
                end
                forwardVec = [cosd(carPose.Yaw), sind(carPose.Yaw), 0];
                app.car.Velocity = forwardVec * app.currentSpeed;
                advance(app.scenario);
                app.frameCount = app.frameCount + 1;
                if mod(app.frameCount, 60) == 0
                    reason = app.determineSpeedReason(curvature, obsDist, obsType, app.obstacleDetectionRange);
                    app.log(sprintf("Status: Speed=%.1f m/s (%.1f km/h) | %s", app.currentSpeed, app.currentSpeed * 3.6, reason));
                end
                dThis = norm(carPos - app.lastPosition);
                app.totalDistance = app.totalDistance + dThis;
                app.lastPosition = carPos;
                if isvalid(app.vehiclePlotHandle)
                    set(app.vehiclePlotHandle, 'XData', carPos(1), 'YData', carPos(2));
                else
                    app.vehiclePlotHandle = plot(app.UIAxes, carPos(1), carPos(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
                end
                app.exportFrame();
            catch ME
                app.log("Timer error: " + ME.message);
                app.stopAndCleanup();
                app.logFrameToJSON();
            end
        end

        function speed = calculateDynamicSpeed(app, curvature, obstacleDistance, obstacleType)
            speed = app.maxSpeed_ms;
            if obstacleDistance < app.obstacleDetectionRange
                obstacleFactor = app.calculateObstacleFactor(obstacleDistance, obstacleType, app.obstacleDetectionRange);
                speed = speed * obstacleFactor;
            end
            if curvature > 0.1
                curvatureFactor = max(0.3, 1 - (curvature * app.curvatureWeight));
                speed = speed * curvatureFactor;
            end
            speed = max(app.minSpeed_ms, min(app.maxSpeed_ms, speed));
        end

        function PauseResumeButtonPushed(app, ~, ~)
            if isempty(app.simTimer) || ~isvalid(app.simTimer)
                app.log("No running simulation to pause/resume.");
                return;
            end
            app.isPaused = ~app.isPaused;
            if app.isPaused
                app.log("Paused.");
            else
                app.log("Resumed.");
            end
        end

        function StopButtonPushed(app, ~, ~)
            app.log("Stop pressed. Cleaning up...");
            app.stopAndCleanup();
        end

        function stopAndCleanup(app)
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer);
                delete(app.simTimer);
                app.simTimer = [];
            end
            if app.frameCount > 0 && app.scenario.SimulationTime > 0
                avgSpeed_kmh = (app.totalDistance / app.scenario.SimulationTime) * 3.6;
                app.log(sprintf('=== SIMULATION COMPLETE ==='));
                app.log(sprintf('Frames: %d | Time: %.1fs | Distance: %.0fm | Avg Speed: %.0f km/h', app.frameCount, app.scenario.SimulationTime, app.totalDistance, avgSpeed_kmh));
            else
                app.log("Simulation stopped.");
            end
        end

        function log(app, msg)
            try
                existing = app.LogTextArea.Value;
                if ischar(existing)
                    existing = string(existing);
                end
                timestamp = datestr(now, 'HH:MM:SS');
                entry = string(sprintf('[%s] %s', timestamp, msg));
                app.LogTextArea.Value = [existing; entry];
                drawnow;
            catch
                try
                    fprintf('[%s] %s\n', datestr(now, 'HH:MM:SS'), char(msg));
                catch
                end
            end
        end

        function plotObstacles(app)
            cla(app.UIAxes);
            if ~isempty(app.scenario)
                app.plotDetailedRoads(app.scenario);
            end
            plot(app.UIAxes, app.carPathWaypoints(:, 1), app.carPathWaypoints(:, 2), 'r--', 'LineWidth', 2);
            if isempty(app.obstaclePositions)
                return;
            end
            for i = 1:size(app.obstaclePositions, 1)
                pos = app.obstaclePositions(i, :);
                typ = app.obstacleTypes(i);
                if typ == 2
                    plot(app.UIAxes, pos(1), pos(2), 'kx', 'MarkerSize', 15, 'LineWidth', 3);
                elseif typ == 3
                    plot(app.UIAxes, pos(1), pos(2), '^', 'MarkerSize', 15, 'MarkerFaceColor', [1 0.8 0], 'MarkerEdgeColor', 'black', 'LineWidth', 2);
                end
            end
            cp = app.car.Position;
            app.vehiclePlotHandle = plot(app.UIAxes, cp(1), cp(2), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
            drawnow;
        end

        function [positions, types, props] = createObstacles(app, npotholes, nbarricades)
            positions = [];
            types = [];
            props = [];
            L = size(app.carPathWaypoints, 1);
            rng('shuffle');
            for i = 1:npotholes
                idx = randi([max(1, round(L * 0.2)), max(2, round(L * 0.9))]);
                pos = [app.carPathWaypoints(idx, :) 0];
                pos(1:2) = pos(1:2) + (rand(1, 2) - 0.5) * 2;
                positions = [positions; pos];
                types = [types; 2];
                props = [props; struct('severity', randi([1 3]), 'width', 1.2, 'active', true)];
            end
            for i = 1:nbarricades
                idx = randi([max(1, round(L * 0.3)), max(2, round(L * 0.95))]);
                pos = [app.carPathWaypoints(idx, :) 0];
                lateral = (rand - 0.5) * 4;
                pos2 = pos + [lateral lateral * 0.2 0];
                positions = [positions; pos2];
                types = [types; 3];
                props = [props; struct('severity', 2, 'width', 3.0, 'active', true)];
            end
        end

        function plotDetailedRoads(app, scenario)
            ax = app.UIAxes;
            hold(ax, 'on');
            roadColor = [0.4 0.4 0.4];
            centerLineColor = [1 1 0];
            for i = 1:length(scenario.RoadSegments)
                roadCenters = scenario.RoadSegments(i).RoadCenters;
                plot(ax, roadCenters(:, 1), roadCenters(:, 2), '-', 'Color', roadColor, 'LineWidth', 6);
                plot(ax, roadCenters(:, 1), roadCenters(:, 2), '-', 'Color', centerLineColor, 'LineWidth', 1.5);
            end
        end

        function [x, y] = deg2utm(~, lat, lon)
            R = 6371000;
            lat0 = mean(lat);
            lon0 = mean(lon);
            x = R * (deg2rad(lon - lon0)) .* cosd(lat0);
            y = R * deg2rad(lat - lat0);
        end

        function [distance, obstacleType, idx] = findNearestObstacle(~, carPosition, obstaclePositions, obstacleTypes)
            if isempty(obstaclePositions)
                distance = inf;
                obstacleType = 0;
                idx = 0;
                return;
            end
            dvec = sqrt(sum((obstaclePositions(:, 1:2) - carPosition(1:2)).^2, 2));
            [distance, idx] = min(dvec);
            obstacleType = obstacleTypes(idx);
        end

        function factor = calculateObstacleFactor(~, dist, type, range)
            if dist > range
                factor = 1.0;
                return;
            end
            switch type
                case 2
                    severityMultiplier = 0.4;
                case 3
                    severityMultiplier = 0.2;
                otherwise
                    severityMultiplier = 0.5;
            end
            proximityFactor = dist / range;
            factor = severityMultiplier + (1 - severityMultiplier) * proximityFactor;
            factor = max(0.1, min(1.0, factor));
        end

        function curvature = calculatePathCurvature(~, pathWaypoints, idx)
            lookahead = min(idx + 5, size(pathWaypoints, 1));
            future = min(idx + 10, size(pathWaypoints, 1));
            if idx >= size(pathWaypoints, 1) - 10
                curvature = 0;
                return;
            end
            v1 = pathWaypoints(lookahead, :) - pathWaypoints(idx, :);
            v2 = pathWaypoints(future, :) - pathWaypoints(lookahead, :);
            if norm(v1) < eps || norm(v2) < eps
                curvature = 0;
                return;
            end
            cosA = dot(v1, v2) / (norm(v1) * norm(v2));
            cosA = max(-1, min(1, cosA));
            angle = acosd(cosA);
            curvature = abs(angle) / 90;
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

        function [closestPoint, d, idx] = snap2path(~, pathWaypoints, point)
            if size(point, 1) > size(point, 2)
                point = point';
            end
            if size(pathWaypoints, 2) > 2
                pathWaypoints = pathWaypoints(:, 1:2);
            end
            if size(point, 2) == 1 && size(pathWaypoints, 2) == 2
                error('Point and path dimensions incompatible');
            elseif size(point, 2) == 2 && size(pathWaypoints, 2) == 2
                dists = sqrt(sum((pathWaypoints - point).^2, 2));
            else
                minDim = min(size(point, 2), size(pathWaypoints, 2));
                dists = sqrt(sum((pathWaypoints(:, 1:minDim) - point(1:minDim)).^2, 2));
            end
            [d, idx] = min(dists);
            closestPoint = pathWaypoints(idx, :);
        end
    end

    methods (Access = private)
        function createComponents(app)
            app.UIFigure = uifigure('Name', 'TrueGrid Simulator', 'Position', [100 100 1200 720]);
            app.LoadMapButton = uibutton(app.UIFigure, 'push', 'Position', [20 700 80 30], 'Text', 'Load OSM', 'ButtonPushedFcn', @(btn, event)app.LoadMapButtonPushed(btn, event));
            app.CreateSimpleMapButton = uibutton(app.UIFigure, 'push', 'Position', [110 700 120 30], 'Text', 'Create Simple Map', 'ButtonPushedFcn', @(btn, event)app.CreateSimpleMapButtonPushed(btn, event));
            uilabel(app.UIFigure, 'Position', [240 705 30 20], 'Text', 'Lat');
            app.LatField = uieditfield(app.UIFigure, 'numeric', 'Position', [270 705 70 22]);
            uilabel(app.UIFigure, 'Position', [350 705 30 20], 'Text', 'Lon');
            app.LonField = uieditfield(app.UIFigure, 'numeric', 'Position', [380 705 70 22]);
            uilabel(app.UIFigure, 'Position', [460 705 50 20], 'Text', 'Radius');
            app.RadiusField = uieditfield(app.UIFigure, 'numeric', 'Position', [510 705 50 22]);
            app.FetchMapButton = uibutton(app.UIFigure, 'push', 'Position', [570 700 120 30], 'Text', 'Fetch from OSM', 'ButtonPushedFcn', @(btn, event)app.FetchMapButtonPushed(btn, event));
            uilabel(app.UIFigure, 'Position', [20 660 120 20], 'Text', 'Num Potholes');
            app.NumPotholesSpinner = uispinner(app.UIFigure, 'Position', [140 660 80 22], 'Limits', [0 50], 'Value', 2);
            uilabel(app.UIFigure, 'Position', [240 660 120 20], 'Text', 'Num Barricades');
            app.NumBarricadesSpinner = uispinner(app.UIFigure, 'Position', [360 660 80 22], 'Limits', [0 50], 'Value', 1);
            uilabel(app.UIFigure, 'Position', [20 620 100 20], 'Text', 'Base Speed (m/s)');
            app.BaseSpeedEdit = uieditfield(app.UIFigure, 'numeric', 'Position', [140 620 80 22], 'Value', 15);
            app.FollowCameraCheck = uicheckbox(app.UIFigure, 'Position', [260 620 120 22], 'Text', 'Follow Camera', 'Value', false);
            app.StartButton = uibutton(app.UIFigure, 'push', 'Position', [20 580 80 30], 'Text', 'Start', 'ButtonPushedFcn', @(btn, event)app.StartButtonPushed(btn, event));
            app.PauseResumeButton = uibutton(app.UIFigure, 'push', 'Position', [110 580 100 30], 'Text', 'Pause/Resume', 'ButtonPushedFcn', @(btn, event)app.PauseResumeButtonPushed(btn, event));
            app.StopButton = uibutton(app.UIFigure, 'push', 'Position', [220 580 80 30], 'Text', 'Stop', 'ButtonPushedFcn', @(btn, event)app.StopButtonPushed(btn, event));
            app.UIAxes = uiaxes(app.UIFigure, 'Position', [20 20 950 540]);
            title(app.UIAxes, 'Enhanced Vehicle Simulation');
            xlabel(app.UIAxes, 'X');
            ylabel(app.UIAxes, 'Y');
            app.LogTextArea = uitextarea(app.UIFigure, 'Position', [1000 20 180 710], 'Editable', 'off');
        end
    end

    methods (Access = public)
        function app = truegrid_sim
            createComponents(app);
            startup(app);
        end

        function startSim(app)
            try
                app.CreateSimpleMapButtonPushed([], []);
            catch
                try
                    app.createSimpleTestMap();
                catch
                end
            end
            pause(0.05);
            try
                app.StartButtonPushed([], []);
            catch
            end
        end

        function delete(app)
            if ~isempty(app.simTimer) && isvalid(app.simTimer)
                stop(app.simTimer);
                delete(app.simTimer);
            end
            try
                delete(app.UIFigure);
            catch
            end
        end
    end
end
