function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.
M=map.occgrid(:);
list=1:numel(map.occgrid);
obst=list(M==1);
obs=map.indToXYZ(obst(:));
scatter3(obs(:,1),obs(:,2),obs(:,3),'black*');hold on;
scatter3(path(:,1),path(:,2),path(:,3),'b*');hold on
xlabel('xaxis');ylabel('yaxis');zlabel('zaxis')

end