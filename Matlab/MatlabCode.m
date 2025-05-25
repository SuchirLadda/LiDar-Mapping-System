% Suchir Ladda - 400517569
% 2DX3 Final Project - ToF Data Collection & Visualization
% --- Serial Port Setup ---
availablePorts = serialportlist("available");
serialConnection = serialport(availablePorts(1), 115200, 'Timeout', 120);
flush(serialConnection);

% --- Measurement Parameters ---
depth = 11;                      % Depth levels (rotations)
num_measurements = 32;              % 360° / 32 = 11.25° per sample
measurementMatrix = zeros(0, 3);   % Will hold [distance, angle, depth]

sampleIndex = 0;
totalSamples = depth * num_measurements;

% --- Data Acquisition Loop ---
while sampleIndex < totalSamples
    rawInput = readline(serialConnection);
    
    if isempty(rawInput)
        break;
    end

    parsed = processSerialData(rawInput);
    cleanSample = parsed([2, 3, 4]);   % Extract [distance, angle, depth]

    disp(cleanSample);                % Debug output
    measurementMatrix = [measurementMatrix; cleanSample]; %#ok<AGROW>
    
    sampleIndex = sampleIndex + 1;
end

% --- Transform to Cartesian Coordinates ---
% Transpose so each row represents a variable
rawData = measurementMatrix.';

% Convert polar (angle, radius, depth) to Cartesian (x, y, z)
[thetaRad, r, zDepth] = deal(rawData(2,:) * pi/180, rawData(1,:), rawData(3,:));
[x, y, z] = pol2cart(thetaRad, r, zDepth);

% Reorient axes to plot along YZ plane
rotatedCoords = [z; y; x];

% --- 3D Visualization ---
figure;
scatter3(rotatedCoords(1,:), rotatedCoords(2,:), rotatedCoords(3,:), 60, 'filled');
hold on;

% --- Connect Points Within Each Slice ---
for slice = 1:depth
    baseIdx = (slice - 1) * num_measurements;
    for j = 1:num_measurements - 1
        idx = baseIdx + j;
        plot3(rotatedCoords(1,idx:idx+1), rotatedCoords(2,idx:idx+1), rotatedCoords(3,idx:idx+1), 'b-');
    end
    % Connect final point to starting point to close the ring
    plot3([rotatedCoords(1,baseIdx+1), rotatedCoords(1,baseIdx+num_measurements)], ...
          [rotatedCoords(2,baseIdx+1), rotatedCoords(2,baseIdx+num_measurements)], ...
          [rotatedCoords(3,baseIdx+1), rotatedCoords(3,baseIdx+num_measurements)], 'b-');
end

% --- Connect Vertices Across Depths ---
for d = 1:depth - 1
    for p = 1:num_measurements
        idx1 = (d - 1) * num_measurements + p;
        idx2 = d * num_measurements + p;
        plot3([rotatedCoords(1,idx1), rotatedCoords(1,idx2)], ...
              [rotatedCoords(2,idx1), rotatedCoords(2,idx2)], ...
              [rotatedCoords(3,idx1), rotatedCoords(3,idx2)], 'k-');
    end
end

% --- Labels and Display ---
hold off;
title('Suchir Ladda - 2DX3 Final Project: 3D Spatial Mapping');
xlabel('Depth Axis (X)');
ylabel('Sensor Sweep (Y)');
zlabel('Distance (Z)');
grid on;
view(3);

% --- Data Parser Function ---
function parsed = processSerialData(serialLine)
    disp("RAW INPUT → " + serialLine); % Print incoming line for reference
    numbers = sscanf(serialLine, '%f,%f,%f,%f,%f');

    % Extract values with inline conversion of angle
    check = numbers(1);
    dist = numbers(2);
    angleDeg = numbers(3) * 11.25 / 16;
    slice = numbers(4);
    spadVal = numbers(5);

    parsed = [check, dist, angleDeg, slice, spadVal];
end