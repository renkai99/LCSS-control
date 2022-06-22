function [At, b0] = compute_approximate_union(avg_theta, vertex)

K_initial = size(vertex,2);

%% Obstacle sets
% load('Saved_data/w_truck_poses.mat');

At = [eye(2,2); -eye(2,2)] * [cos(avg_theta), sin(avg_theta); -sin(avg_theta), cos(avg_theta)];

% vertex enumeration:
% tic;
% b0 = At*vertex(1:2,1);
% for k=1:K_initial
% %     b0 = max(b0, At*vertex(1:2,k));
%     b1 = max(At*vertex(1:2,k), At*vertex(3:4,k));
%     b2 = max(At*vertex(5:6,k), At*vertex(7:8,k));
%     b0 = max(b0,b1); 
%     b0 = max(b0,b2);
% end

a0 = max(At * vertex(1:2, :), [], 2);
a1 = max(At * vertex(3:4, :), [], 2);
a2 = max(At * vertex(5:6, :), [], 2);
a3 = max(At * vertex(7:8, :), [], 2);

b0 = max([a0 a1 a2 a3], [], 2);

% toc(tic) % very efficient if vertexes are available.


% Obar = Polyhedron(At, b0);
% Obar.plot('wire', true, 'linewidth', 2)
% axis equal
% axis([2.5 45 -15 params.lane+1])
end
