clear; close all; clc;

followCamera = false;
zoom_level = 70;

[scriptPath, ~, ~] = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptPath);
addpath(genpath(projectRoot));
dataFile = fullfile(projectRoot, 'data', 'SilkBoardScenario.mat');
fprintf('Loading scenario...\n');
load(dataFile, 'scenario');

roadSeg1 = scenario.RoadSegments(4);
roadSeg2 = scenario.RoadSegments(5);
carPathWaypoints = [roadSeg1.RoadCenters; roadSeg2.RoadCenters];
fprintf('Path created with %d waypoints\n', size(carPathWaypoints, 1));

carSpeed = 15;
car = vehicle(scenario, 'ClassID', 1, 'Position', [carPathWaypoints(1,1:2) 0]);
path(car, carPathWaypoints, carSpeed);

obstaclePositions = [];
obstacleTypes = [];

pothole1Pos = addPothole(scenario, roadSeg1, 50);
pothole2Pos = addPothole(scenario, roadSeg2, 60);
obstaclePositions = [obstaclePositions; pothole1Pos; pothole2Pos];
obstacleTypes = [obstacleTypes; 2; 2];

barricade1Pos = addBarricade(scenario, roadSeg1, 100, -1.5);
obstaclePositions = [obstaclePositions; barricade1Pos];
obstacleTypes = [obstacleTypes; 3];

fprintf('Added %d obstacles to scenario\n', length(obstacleTypes));

fig = figure(1);
clf(fig);
set(fig, 'Position', [100, 100, 1200, 800]);
set(fig, 'Name', 'Dynamic Speed Vehicle Simulation with Obstacle Avoidance');
set(fig, 'NumberTitle', 'off');
set(fig, 'Color', 'white');

hold on;
axis equal;
grid on;
xlabel('X Position (m)', 'FontSize', 12);
ylabel('Y Position (m)', 'FontSize', 12);

plotDetailedRoads(scenario);

plot(carPathWaypoints(:,1), carPathWaypoints(:,2), ...
    'r--', 'LineWidth', 2, 'DisplayName', 'Planned Path');

carStartPosition = scenario.Actors(1).Position;
hVehicle = plot(carStartPosition(1), carStartPosition(2), 'bo', ...
    'MarkerSize', 15, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black', 'LineWidth', 2);

staticObstacles = scenario.Actors(2:end);
for i = 1:length(staticObstacles)
    actor = staticObstacles(i);
    pos = actor.Position;
    
    if actor.ClassID == 2
        plot(pos(1), pos(2), 'kx', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Pothole');
    elseif actor.ClassID == 3
        plot(pos(1), pos(2), '^', 'MarkerSize', 15, 'LineWidth', 2, ...
            'MarkerFaceColor', 'yellow', 'MarkerEdgeColor', 'black', 'DisplayName', 'Barricade');
    end
end

legend({'Planned Path', 'Vehicle', 'Pothole', 'Barricade'}, ...
    'Location', 'northeast', 'FontSize', 10);

fprintf('Plot setup complete\n');

fprintf('Starting animation with dynamic speed control considering obstacles...\n');

frameCount = 0;
totalDistance = 0;
lastPosition = carStartPosition;

maxSpeed_ms = 15;
minSpeed_ms = 3;
obstacleDetectionRange = 25;
curvatureWeight = 3;
obstacleWeight = 8;

while advance(scenario)
    frameCount = frameCount + 1;
    
    poses = actorPoses(scenario);
    if isempty(poses) || length(poses) < 1
        fprintf('No poses available at frame %d\n', frameCount);
        break;
    end
    
    carPose = poses(1);
    carPosition = carPose.Position;
    carVelocity = carPose.Velocity;
    
    distanceThisFrame = norm(carPosition - lastPosition);
    totalDistance = totalDistance + distanceThisFrame;
    lastPosition = carPosition;
    
    [~, ~, closestPathIdx] = snap2path(carPathWaypoints, carPosition');
    
    curvature = calculatePathCurvature(carPathWaypoints, closestPathIdx);
    curvatureFactor = 1 / (1 + curvatureWeight * curvature);
    
    [obstacleDistance, obstacleType, nearestObstacle] = findNearestObstacle(carPosition, obstaclePositions, obstacleTypes);
    obstacleFactor = calculateObstacleFactor(obstacleDistance, obstacleType, obstacleDetectionRange);
    
    combinedFactor = min(curvatureFactor, obstacleFactor);
    targetSpeed_ms = minSpeed_ms + (maxSpeed_ms - minSpeed_ms) * combinedFactor;
    
    targetSpeed_ms = max(minSpeed_ms, min(maxSpeed_ms, targetSpeed_ms));
    
    remainingPath = carPathWaypoints(closestPathIdx:end, :);
    if size(remainingPath, 1) >= 2
        path(car, remainingPath, targetSpeed_ms);
    else
        fprintf('Approaching end of path at frame %d\n', frameCount);
        break;
    end
    
    if isvalid(hVehicle) && isgraphics(hVehicle)
        set(hVehicle, 'XData', carPosition(1), 'YData', carPosition(2));
    else
        hVehicle = plot(carPosition(1), carPosition(2), 'bo', ...
            'MarkerSize', 15, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
    end
    
    if followCamera
        xlim([carPosition(1)-zoom_level, carPosition(1)+zoom_level]);
        ylim([carPosition(2)-zoom_level, carPosition(2)+zoom_level]);
    end
    
    actualSpeed_ms = norm(carVelocity);
    actualSpeed_kmh = actualSpeed_ms * 3.6;
    targetSpeed_kmh = targetSpeed_ms * 3.6;
    
    speedReason = determineSpeedReason(curvature, obstacleDistance, obstacleType, obstacleDetectionRange);
    
    title(sprintf('Time: %.1fs | Target: %.0f km/h | Actual: %.0f km/h | %s | Dist: %.0fm', ...
        scenario.SimulationTime, targetSpeed_kmh, actualSpeed_kmh, speedReason, totalDistance), ...
        'FontSize', 11, 'FontWeight', 'bold');
    
    drawnow;
    pause(0.05);
    
    if mod(frameCount, 30) == 0
        fprintf('Frame %d: Time=%.1fs, Speed=%.0f/%.0f km/h, %s, Obstacle dist=%.1fm\n', ...
            frameCount, scenario.SimulationTime, actualSpeed_kmh, targetSpeed_kmh, ...
            speedReason, obstacleDistance);
    end
    
    if frameCount > 1500
        fprintf('Reached frame limit\n');
        break;
    end
end

avgSpeed_kmh = (totalDistance / scenario.SimulationTime) * 3.6;
title(sprintf('Simulation Complete | Total: %.1fs | Distance: %.0fm | Avg Speed: %.0f km/h', ...
    scenario.SimulationTime, totalDistance, avgSpeed_kmh), ...
    'FontSize', 12, 'Color', 'darkgreen', 'FontWeight', 'bold');

fprintf('\n=== SIMULATION COMPLETE ===\n');
fprintf('Frames: %d | Time: %.1fs | Distance: %.0fm | Avg Speed: %.0f km/h\n', ...
    frameCount, scenario.SimulationTime, totalDistance, avgSpeed_kmh);

function plotDetailedRoads(scenario)
    laneWidth = 3.5;
    roadColor = [0.4 0.4 0.4];
    centerLineColor = [1 1 0];
    laneLineColor = [1 1 1];
    
    for i = 1:length(scenario.RoadSegments)
        road = scenario.RoadSegments(i);
        roadCenters = road.RoadCenters;
        
        if isfield(road, 'Lanes') && ~isempty(road.Lanes)
            numLanes = length(road.Lanes);
            roadWidth = numLanes * laneWidth;
        else
            numLanes = 2;
            roadWidth = numLanes * laneWidth;
        end
        
        plotRoadSurface(roadCenters, roadWidth, roadColor);
        plotLaneMarkings(roadCenters, roadWidth, numLanes, centerLineColor, laneLineColor);
    end
end

function plotRoadSurface(roadCenters, roadWidth, roadColor)
    try
        [leftBoundary, rightBoundary] = offsetRoadCenterline(roadCenters, roadWidth/2);
        
        if ~isempty(leftBoundary) && ~isempty(rightBoundary)
            surfaceVertices = [leftBoundary; flipud(rightBoundary)];
            fill(surfaceVertices(:,1), surfaceVertices(:,2), roadColor, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
        end
    catch
        plot(roadCenters(:,1), roadCenters(:,2), 'Color', roadColor, 'LineWidth', 12);
    end
end

function plotLaneMarkings(roadCenters, roadWidth, numLanes, centerLineColor, laneLineColor)
    try
        plot(roadCenters(:,1), roadCenters(:,2), '-', 'Color', centerLineColor, 'LineWidth', 1.5);
        
        if numLanes > 1
            laneWidth = roadWidth / numLanes;
            for lane = 1:numLanes-1
                offset = -roadWidth/2 + lane * laneWidth;
                [laneLine, ~] = offsetRoadCenterline(roadCenters, offset);
                if ~isempty(laneLine)
                    plotDashedLine(laneLine, laneLineColor, 3, 6);
                end
            end
        end
        
        [leftEdge, rightEdge] = offsetRoadCenterline(roadCenters, roadWidth/2);
        if ~isempty(leftEdge) && ~isempty(rightEdge)
            plot(leftEdge(:,1), leftEdge(:,2), '-', 'Color', laneLineColor, 'LineWidth', 1.5);
            plot(rightEdge(:,1), rightEdge(:,2), '-', 'Color', laneLineColor, 'LineWidth', 1.5);
        end
    catch
        plot(roadCenters(:,1), roadCenters(:,2), '-', 'Color', centerLineColor, 'LineWidth', 1.5);
    end
end

function [leftOffset, rightOffset] = offsetRoadCenterline(roadCenters, offsetDistance)
    leftOffset = [];
    rightOffset = [];
    
    try
        for i = 1:size(roadCenters, 1)-1
            dx = roadCenters(i+1,1) - roadCenters(i,1);
            dy = roadCenters(i+1,2) - roadCenters(i,2);
            
            length_segment = sqrt(dx^2 + dy^2);
            if length_segment > eps
                perp_x = -dy / length_segment;
                perp_y = dx / length_segment;
                
                leftOffset = [leftOffset; roadCenters(i,1) + perp_x * offsetDistance, roadCenters(i,2) + perp_y * offsetDistance];
                rightOffset = [rightOffset; roadCenters(i,1) - perp_x * offsetDistance, roadCenters(i,2) - perp_y * offsetDistance];
            end
        end
        
        if size(roadCenters, 1) > 1
            i = size(roadCenters, 1);
            dx = roadCenters(i,1) - roadCenters(i-1,1);
            dy = roadCenters(i,2) - roadCenters(i-1,2);
            length_segment = sqrt(dx^2 + dy^2);
            if length_segment > eps
                perp_x = -dy / length_segment;
                perp_y = dx / length_segment;
                
                leftOffset = [leftOffset; roadCenters(i,1) + perp_x * offsetDistance, roadCenters(i,2) + perp_y * offsetDistance];
                rightOffset = [rightOffset; roadCenters(i,1) - perp_x * offsetDistance, roadCenters(i,2) - perp_y * offsetDistance];
            end
        end
    catch
    end
end

function plotDashedLine(points, color, dashLength, gapLength)
    try
        if size(points, 1) < 2, return; end
        
        totalLength = 0;
        for i = 1:size(points, 1)-1
            segmentLength = norm(points(i+1,:) - points(i,:));
            totalLength = totalLength + segmentLength;
        end
        
        currentPos = 0;
        isDrawing = true;
        
        while currentPos < totalLength
            if isDrawing
                [startPoint, startIdx] = interpolateAlongPath(points, currentPos);
                [endPoint, endIdx] = interpolateAlongPath(points, min(currentPos + dashLength, totalLength));
                
                if startIdx == endIdx
                    plot([startPoint(1), endPoint(1)], [startPoint(2), endPoint(2)], '-', 'Color', color, 'LineWidth', 1.5);
                else
                    dashPoints = [startPoint; points(startIdx+1:endIdx-1, :); endPoint];
                    plot(dashPoints(:,1), dashPoints(:,2), '-', 'Color', color, 'LineWidth', 1.5);
                end
                
                currentPos = currentPos + dashLength;
                isDrawing = false;
            else
                currentPos = currentPos + gapLength;
                isDrawing = true;
            end
        end
    catch
        plot(points(:,1), points(:,2), '-', 'Color', color, 'LineWidth', 1.5);
    end
end

function [point, segmentIndex] = interpolateAlongPath(points, distance)
    accumulatedDistance = 0;
    
    for i = 1:size(points, 1)-1
        segmentVector = points(i+1,:) - points(i,:);
        segmentLength = norm(segmentVector);
        
        if accumulatedDistance + segmentLength >= distance
            t = (distance - accumulatedDistance) / segmentLength;
            point = points(i,:) + t * segmentVector;
            segmentIndex = i;
            return;
        end
        
        accumulatedDistance = accumulatedDistance + segmentLength;
    end
    
    point = points(end,:);
    segmentIndex = size(points, 1) - 1;
end

function curvature = calculatePathCurvature(pathWaypoints, currentIndex)
    lookahead_idx = min(currentIndex + 5, size(pathWaypoints, 1));
    future_idx = min(currentIndex + 10, size(pathWaypoints, 1));
    
    if currentIndex >= size(pathWaypoints, 1) - 10
        curvature = 0;
        return;
    end
    
    vec1 = pathWaypoints(lookahead_idx,:) - pathWaypoints(currentIndex,:);
    vec2 = pathWaypoints(future_idx,:) - pathWaypoints(lookahead_idx,:);
    
    if norm(vec1) < eps || norm(vec2) < eps
        curvature = 0;
        return;
    end
    
    cosAngle = dot(vec1, vec2) / (norm(vec1) * norm(vec2));
    cosAngle = max(-1, min(1, cosAngle));
    angle = acosd(cosAngle);
    curvature = abs(angle) / 90;
end

function [distance, obstacleType, obstacleIdx] = findNearestObstacle(carPosition, obstaclePositions, obstacleTypes)
    if isempty(obstaclePositions)
        distance = inf;
        obstacleType = 0;
        obstacleIdx = 0;
        return;
    end
    
    distances = sqrt(sum((obstaclePositions(:,1:2) - carPosition(1:2)).^2, 2));
    [distance, obstacleIdx] = min(distances);
    obstacleType = obstacleTypes(obstacleIdx);
end

function factor = calculateObstacleFactor(obstacleDistance, obstacleType, detectionRange)
    if obstacleDistance > detectionRange
        factor = 1.0;
        return;
    end
    
    switch obstacleType
        case 2
            severityMultiplier = 0.6;
        case 3
            severityMultiplier = 0.3;
        otherwise
            severityMultiplier = 0.5;
    end
    
    proximityFactor = obstacleDistance / detectionRange;
    factor = severityMultiplier + (1 - severityMultiplier) * proximityFactor;
    factor = max(0.1, min(1.0, factor));
end

function reason = determineSpeedReason(curvature, obstacleDistance, obstacleType, detectionRange)
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

function [closestPoint, distance, idx] = snap2path(pathWaypoints, point)
    distances = sqrt(sum((pathWaypoints - point').^2, 2));
    [distance, idx] = min(distances);
    closestPoint = pathWaypoints(idx,:);
end

function position = addPothole(scenario, roadSegment, positionAlongRoad)
    [position, ~] = findPositionOnPath(roadSegment.RoadCenters, positionAlongRoad);
    potholeDims = struct('Length', 1.2, 'Width', 0.7, 'Height', 0.05);
    actor(scenario, 'ClassID', 2, 'Length', potholeDims.Length, ...
        'Width', potholeDims.Width, 'Height', potholeDims.Height, ...
        'Position', position, 'PlotColor', [0.4 0.4 0.4]);
end

function position = addBarricade(scenario, roadSegment, positionAlongRoad, lateralOffset)
    [centerPosition, directionVector] = findPositionOnPath(roadSegment.RoadCenters, positionAlongRoad);
    perpendicularVector = [-directionVector(2), directionVector(1), 0];
    position = centerPosition + lateralOffset * perpendicularVector;
    yaw = atan2d(directionVector(2), directionVector(1));
    barricadeDims = struct('Length', 2.5, 'Width', 0.5, 'Height', 1.5);
    actor(scenario, 'ClassID', 3, 'Length', barricadeDims.Length, ...
        'Width', barricadeDims.Width, 'Height', barricadeDims.Height, ...
        'Position', position, 'Yaw', yaw, 'PlotColor', [1 0.8 0]);
end

function [position, directionVector] = findPositionOnPath(vertices, positionAlongRoad)
    totalLength = 0;
    directionVector = [1 0 0];
    for i = 1:(size(vertices,1) - 1)
        p1 = vertices(i,:); p2 = vertices(i+1,:);
        segmentVector = p2 - p1;
        segmentLength = norm(segmentVector);
        
        if (totalLength + segmentLength) >= positionAlongRoad
            remainingDist = positionAlongRoad - totalLength;
            directionVector = segmentVector / segmentLength;
            position = p1 + remainingDist * directionVector;
            return;
        end
        totalLength = totalLength + segmentLength;
    end
    position = vertices(end,:);
    if size(vertices,1) > 1
        directionVector = (vertices(end,:) - vertices(end-1,:)) / norm(vertices(end,:) - vertices(end-1,:));
    end
end
