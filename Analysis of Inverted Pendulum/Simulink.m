% Build Simulink model: Furuta pendulum with State-Space plant, LQR, and observer
% Usage:
%   1) Set K and L below (or compute them in MATLAB).
%   2) Run this script.
%   3) Open and simulate the generated model.

clear; clc;

%% Plant matrices (from assignment)
A = [0 0 1 0;
     0 0 0 1;
     0 123.98 -1.577 0;
     0 111.62 -0.7253 0];

B = [0; 0; 56.38; 25.98];

% Measured outputs: theta and alpha only
C = [1 0 0 0;
     0 1 0 0];

D = [0; 0];

%% Controller/observer gains
% Replace these with your assignment values.
% Example placeholders (do NOT submit placeholders):
K = [-3.1623 58.0056 -1.9949 7.3754];          % 1x4
L = zeros(4,2);         % 4x2

% Validation
assert(all(size(K) == [1 4]), 'K must be 1x4.');
assert(all(size(L) == [4 2]), 'L must be 4x2.');

%% Model name
mdl = 'furuta_lqr_observer';
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

new_system(mdl);
open_system(mdl);

%% Add blocks
% Sources
add_block('simulink/Sources/Constant', [mdl '/r'], ...
    'Value', '0', 'Position', [30 70 80 100]);

% Observer state-space: xhat_dot = (A-LC)xhat + [B L]*[u; y], yhat = I*xhat
add_block('simulink/Continuous/State-Space', [mdl '/Observer'], ...
    'A', 'A - L*C', ...
    'B', '[B L]', ...
    'C', 'eye(4)', ...
    'D', 'zeros(4,3)', ...
    'Position', [500 210 670 300]);

% LQR control u = -K*xhat + r
add_block('simulink/Math Operations/Gain', [mdl '/LQR -K'], ...
    'Gain', '-K', ...
    'Multiplication', 'Matrix(K*u)', ...
    'Position', [720 230 820 270]);

add_block('simulink/Math Operations/Sum', [mdl '/u sum'], ...
    'Inputs', '++', ...
    'Position', [860 235 885 265]);

% Plant
add_block('simulink/Continuous/State-Space', [mdl '/Plant'], ...
    'A', 'A', ...
    'B', 'B', ...
    'C', 'C', ...
    'D', 'D', ...
    'Position', [930 210 1080 300]);

% Mux y into observer input [u; y]
add_block('simulink/Signal Routing/Mux', [mdl '/Mux u_y'], ...
    'Inputs', '3', ...
    'Position', [430 230 455 290]);

% Output selection for scopes
add_block('simulink/Signal Routing/Demux', [mdl '/Demux y'], ...
    'Outputs', '2', ...
    'Position', [1120 225 1145 285]);

% Scopes
add_block('simulink/Sinks/Scope', [mdl '/Scope theta'], ...
    'Position', [1190 210 1240 240]);

add_block('simulink/Sinks/Scope', [mdl '/Scope alpha'], ...
    'Position', [1190 260 1240 290]);

add_block('simulink/Sinks/Scope', [mdl '/Scope u'], ...
    'Position', [930 90 980 120]);

%% Wire blocks
% Observer output xhat -> LQR
add_line(mdl, 'Observer/1', 'LQR -K/1', 'autorouting', 'on');

% Reference + LQR output -> u
add_line(mdl, 'r/1', 'u sum/1', 'autorouting', 'on');
add_line(mdl, 'LQR -K/1', 'u sum/2', 'autorouting', 'on');

% u -> plant and scope
add_line(mdl, 'u sum/1', 'Plant/1', 'autorouting', 'on');
add_line(mdl, 'u sum/1', 'Scope u/1', 'autorouting', 'on');

% Plant y -> demux -> scopes
add_line(mdl, 'Plant/1', 'Demux y/1', 'autorouting', 'on');
add_line(mdl, 'Demux y/1', 'Scope theta/1', 'autorouting', 'on');
add_line(mdl, 'Demux y/2', 'Scope alpha/1', 'autorouting', 'on');

% Observer input Mux = [u; y]
add_line(mdl, 'u sum/1', 'Mux u_y/1', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'Mux u_y/2', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'Mux u_y/3', 'autorouting', 'on');
add_line(mdl, 'Mux u_y/1', 'Observer/1', 'autorouting', 'on');

%% Simulation settings
set_param(mdl, 'StopTime', '10');
set_param(mdl, 'Solver', 'ode45');

save_system(mdl);
open_system(mdl);

disp('Model created: furuta_lqr_observer.slx');
disp('Set your K and L values in the script, rerun if needed, then simulate.');
