profile on
close all;
%clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map = load_map('maps/map1.txt', 0.15, 1, 0.28);
start = {[0 20 3]};
stop  = {[5 -5 1]};
% start  = {[4.5  15 3.0]};
% stop  = {[4.5  12  3.0]};
nquad = length(start);
for qn = 1:nquad
   path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
   plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
