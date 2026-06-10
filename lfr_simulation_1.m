clc; clear; close all;

% ============================================================
%   PID CONSTANTS  (same as your Arduino)
% ============================================================
KP         = 29.0;
KI         = 0.02;
KD         = 15.0;
BASE_SPEED = 150;
MAX_SPEED  = 200;
MIN_SPEED  = 20;
TURN_SPEED = 80;

% ============================================================
%   SENSOR CONFIG
%   FIX 1: LINE_THRESHOLD = 0.012  (was 0.12)
%          0.12 made ALL 5 sensors trigger always (allBlack)
%          0.012 < sensor_spacing(0.015) = clean 1-2 sensor reads
% ============================================================
SENSOR_WEIGHT  = [-2, -1, 0, 1, 2];
SENSOR_SPACING = 0.015;
SENSOR_OFFSETS = SENSOR_SPACING * [-2, -1, 0, 1, 2];
LINE_WIDTH     = 0.025;   % your actual black tape width in meters
LINE_THRESHOLD = LINE_WIDTH / 2;

% ============================================================
%   FIX 2: 5000 path points (was 1000)
% ============================================================
t      = linspace(0, 2*pi, 5000);
path_x = 2.0 * cos(t);
path_y = 1.0 * sin(t);
N      = length(path_x);
path_theta = atan2(gradient(path_y), gradient(path_x));

% ============================================================
%   FIGURE — single large track window
% ============================================================
fig = figure('Name', 'Line Follower Robot — PID Simulation', ...
             'Position', [80 60 950 720], ...
             'Color', [1 1 1]);

ax = axes('Parent', fig);
hold(ax, 'on'); axis(ax, 'equal');
axis(ax, [-3.0 3.0 -1.9 1.9]);
set(ax, 'Color', [1 1 1], ...
        'XColor', 'none', 'YColor', 'none');

% Track
plot(ax, path_x, path_y, '-', 'LineWidth', 8, 'Color', [0 0 0]);

% Start marker
plot(ax, path_x(1), path_y(1), 'p', ...
     'MarkerSize', 20, 'MarkerFaceColor', [0.2 0.95 0.35], ...
     'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(ax, path_x(1)+0.13, path_y(1)+0.22, 'START', ...
     'Color', [0.2 0.95 0.35], 'FontSize', 11, ...
     'FontWeight', 'bold', 'FontName', 'Courier New');

% Robot trace
h_trace = plot(ax, NaN, NaN, '-', ...
               'Color', [0.25 0.65 1.0 0.45], 'LineWidth', 2);

% Robot body
h_robot = plot(ax, 0, 0, 'o', ...
               'MarkerSize', 24, ...
               'MarkerFaceColor', [0.15 0.55 1.0], ...
               'MarkerEdgeColor', [1 1 1], 'LineWidth', 2.5);

% Direction arrow
h_arrow = quiver(ax, 0, 0, 0.3, 0, 0, ...
                 'Color', [1.0 0.4 0.1], ...
                 'LineWidth', 3.0, 'MaxHeadSize', 3.0);

% 5 sensor squares
h_s = gobjects(1, 5);
for s = 1:5
    h_s(s) = plot(ax, 0, 0, 's', ...
                  'MarkerSize', 11, ...
                  'MarkerFaceColor', [0.8 0.1 0.1], ...
                  'MarkerEdgeColor', 'w', 'LineWidth', 1);
end

h_title = title(ax, 'Initialising...', ...
                'Color', [0.95 0.95 0.95], 'FontSize', 12, ...
                'FontWeight', 'bold', 'FontName', 'Courier New');

% ============================================================
%   ROBOT STATE
%   FIX 3: path-indexed model
%          pidx = float position along path (index)
%          lat  = lateral offset from path centre (metres)
% ============================================================
lastError  = 0;
integral_v = 0;
pidx       = 1.0;
lat        = 0.0;
ADVANCE    = 1.0;
LAT_SCALE  = 0.0001;

trace_x = path_x(1);
trace_y = path_y(1);

fprintf('KP=%.1f  KI=%.2f  KD=%.1f  BASE=%d\n', KP, KI, KD, BASE_SPEED);

% ============================================================
%   MAIN LOOP
% ============================================================
for lap = 1:5
    if ~ishandle(fig), break; end

    for step = 1:N
        if ~ishandle(fig), break; end

        ci = mod(round(pidx) - 1, N) + 1;

        perp_angle = path_theta(ci) + pi/2;
        rx     = path_x(ci)  + lat * cos(perp_angle);
        ry     = path_y(ci)  + lat * sin(perp_angle);
        rtheta = path_theta(ci);

        % Read sensors
        perp       = rtheta + pi/2;
        sensorVals = zeros(1, 5);
        for s = 1:5
            sx = rx + SENSOR_OFFSETS(s) * cos(perp);
            sy = ry + SENSOR_OFFSETS(s) * sin(perp);
            d  = sqrt(min((path_x - sx).^2 + (path_y - sy).^2));
            if d < LINE_THRESHOLD
                sensorVals(s) = 1;
            end
        end

        allWhite_v = all(sensorVals == 0);
        allBlack_v = all(sensorVals == 1);

        % Arduino PID logic
        if allWhite_v
            turn_dir = sign(lastError);
            if turn_dir == 0, turn_dir = 1; end
            lat  = lat - turn_dir * 0.0005;
            lat  = max(-0.07, min(0.07, lat));
            pidx = pidx + 0.3;
            leftPWM  =  TURN_SPEED * turn_dir;
            rightPWM = -TURN_SPEED * turn_dir;

        elseif allBlack_v
            lastError  = 0;
            integral_v = 0;
            pidx = pidx + ADVANCE;
            leftPWM  = BASE_SPEED;
            rightPWM = BASE_SPEED;

        else
            activeSensors = 0;
            weightedSum   = 0;
            for i = 1:5
                if sensorVals(i) == 1
                    weightedSum   = weightedSum + SENSOR_WEIGHT(i);
                    activeSensors = activeSensors + 1;
                end
            end

            if activeSensors == 0
                error_v = lastError;
            else
                error_v = weightedSum / activeSensors;
            end

            integral_v = integral_v + error_v;
            integral_v = max(-100, min(100, integral_v));
            derivative = error_v - lastError;
            correction = (KP * error_v) + (KI * integral_v) + (KD * derivative);
            lastError  = error_v;

            leftPWM  = max(MIN_SPEED, min(MAX_SPEED, BASE_SPEED + correction));
            rightPWM = max(MIN_SPEED, min(MAX_SPEED, BASE_SPEED - correction));

            avg_speed = (leftPWM + rightPWM) / 2;
            pidx = pidx + ADVANCE * (avg_speed / BASE_SPEED);

            lat = lat - error_v * LAT_SCALE * KP;
            lat = max(-0.06, min(0.06, lat));
        end

        if pidx > N, pidx = pidx - N; end

        trace_x(end+1) = rx;
        trace_y(end+1) = ry;

        % Graphics every 3 steps
        if mod(step, 3) == 0
            set(h_robot, 'XData', rx, 'YData', ry);
            set(h_arrow, 'XData', rx, 'YData', ry, ...
                'UData', 0.30*cos(rtheta), ...
                'VData', 0.30*sin(rtheta));

            if length(trace_x) > 1200
                trace_x = trace_x(end-1199:end);
                trace_y = trace_y(end-1199:end);
            end
            set(h_trace, 'XData', trace_x, 'YData', trace_y);

            sx_arr = rx + SENSOR_OFFSETS .* cos(perp);
            sy_arr = ry + SENSOR_OFFSETS .* sin(perp);
            for s = 1:5
                set(h_s(s), 'XData', sx_arr(s), 'YData', sy_arr(s));
                if sensorVals(s) == 1
                    set(h_s(s), 'MarkerFaceColor', [0.1 1.0 0.2]);
                else
                    set(h_s(s), 'MarkerFaceColor', [1.0 0.2 0.2]);
                end
            end

            s_str = num2str(sensorVals, '%d');
            s_str = s_str(~isspace(s_str));
            set(h_title, 'String', sprintf( ...
                'Lap %d/5  |  Step %d  |  Err: %.2f  |  L: %d  R: %d  |  Sensors: %s', ...
                lap, step, lastError, round(leftPWM), round(rightPWM), s_str));

            drawnow;
        end
    end
    fprintf('Lap %d complete | lateral offset: %.4f m\n', lap, lat);
end

fprintf('\nDone. KP=%.1f KI=%.2f KD=%.1f safe for hardware.\n', KP, KI, KD);
