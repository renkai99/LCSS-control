function [ u_star, cost, x_star, A_union, b_union, t_overapprox, vertex, DIAGNOSTIC ] = solve_proposed(params, OV, EV)
%%  Load needed parameters
car.d = params.car.d ;
truck.d = params.truck.d ;
truck.theta0 = params.truck.theta0;
truck.start = params.truck.start ;

T = params.T;
O = params.O;
L = params.L;
nx = params.nx;
nu = params.nu;
nw = params.nw;
Gamma = params.Gamma ;
States_free_init = params.States_free_init;
c1 = params.c1;
c2 = params.c2;
c3 = params.c3;
t1 = params.t1;
t2 = params.t2;
t3 = params.t3;
diag = params.diag;
K = params.K;
N_proposed = params.N_proposed;
x0 = params.x0;

%% Compute the approximation of the union of obstacle sets
% Find vertexes of sampled obstacle sets
draw_sample = 0;

num_OV = length(OV);
vertex = cell(T, max(K), num_OV);
for i=1:num_OV
    num_lat = OV{i}.num_latent;
    for j=1:num_lat
        for t=1:T
            for k=1:size(OV{i}.pred_position{j},1)
                truck_y1 = OV{i}.pred_position{j}(k,t,1);
                truck_y2 = OV{i}.pred_position{j}(k,t,2);
                truck_theta = OV{i}.pred_yaw{j}(k,t);


            if draw_sample == 1
                if rand(1) < 0.01
                    H1 = [eye(2,2); -eye(2,2)] * [cos(truck_theta), sin(truck_theta); -sin(truck_theta), cos(truck_theta)];
                    h1 = [eye(2,2); -eye(2,2)] * [cos(truck_theta), sin(truck_theta); -sin(truck_theta), cos(truck_theta)] * [truck_y1; truck_y2] + 1/2 * [truck.d(1); truck.d(2); truck.d(1); truck.d(2)] + repmat(diag, 4,1);
                    P1 = Polyhedron(H1, h1);
                    hold on
                    P1.plot('alpha', 0.2, 'color', 'black')
                end
            end

            vertex{t,j,i}(1:2,k) = [truck_y1; truck_y2] + 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [-sin(truck_theta); cos(truck_theta)];
            vertex{t,j,i}(3:4,k) = [truck_y1; truck_y2] + 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [sin(truck_theta); -cos(truck_theta)];
            vertex{t,j,i}(5:6,k) = [truck_y1; truck_y2] - 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [-sin(truck_theta); cos(truck_theta)];
            vertex{t,j,i}(7:8,k) = [truck_y1; truck_y2] - 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [sin(truck_theta); -cos(truck_theta)];
%                 vertex{t,j,i}(1:2,k) = [truck_y1; truck_y2];
            end
        end
    end
end

% Cluster the samples
t_overapprox_start = tic;
% num_cluster = K;
% 
% w_vec_N = w_vec(nw * T + (1:3),:); % samples at last time
% [idx_cluster, Centers] = kmeans(w_vec_N', num_cluster); % clustering performed only once at the last prediction time
% if abs(Centers(1,3)) < abs(Centers(2,3)) % cluster 1 corresponds to go straight
% else
%     idx_cluster = 3 - idx_cluster;
% end

A_union = cell(T, max(K), num_OV);
b_union = cell(T, max(K), num_OV);
for i=1:num_OV
    num_lat = OV{i}.num_latent;
    for j=1:num_lat
        for t=1:T
            vertex_k = vertex{t,j,i};
            avg_theta_k = mean(OV{i}.pred_yaw{j}(:,t));
            [A_union_k, b_union_k] = compute_approximate_union(avg_theta_k, vertex_k);
            A_union{t,j,i} = A_union_k;
            b_union{t,j,i} = b_union_k;
%             plot(vertex_k(1,:), vertex_k(2,:),' b.')
%             hold on
%             P1 = Polyhedron(A_union_k, b_union_k);
%             P1.plot('alpha',0.2);
        end
    end
end

t_overapprox = toc(t_overapprox_start);
fprintf(' The overapproximating obstacle set computation: %1.4f sec.\n', t_overapprox);


%% Plotting
plot_enable = 0; % For vehicle =2, two modes
if plot_enable
    veh = 2; 
    time_stamps = [8];
    
    figure(3)
    hold on
    for t=1:length(time_stamps)
        t0 = time_stamps(t);
        subplot(1, length(time_stamps), t);
        hold on;
        plot(OV{veh}.pred_position{1}(:,t0,1), OV{veh}.pred_position{1}(:,t0,2), 'r.')
%         plot(vertex{t, 1, veh}(1,:), vertex{t,1,veh}(2,:), 'r.')
        plot(OV{veh}.pred_position{2}(:,t0,1), OV{veh}.pred_position{2}(:,t0,2), 'g.')
        P1 = Polyhedron(A_union{t0,2,veh}, b_union{t0,2,veh});
        P2 = Polyhedron(A_union{t0,1,veh}, b_union{t0,1,veh});
        P1.plot('alpha', 0.2, 'color', 'black')
        P2.plot('alpha', 0.2, 'color', 'black')
    end
end


%% Motion planning problem
%%% Optimization variables

u = sdpvar(nu*T,1,'full'); % input sequence 
delta = binvar(L*sum(K),T); % binary variables for obstacle 

x_future = States_free_init +  Gamma*u;
big_M = 200; % conservative upper-bound

%%% Constraints
constraints = [];
constraints_obs = [];

x1 = x_future(nx+1:nx:end,:);
x2 = x_future(nx+2:nx:end,:);
x3 = x_future(nx+3:nx:end,:);
x4 = x_future(nx+4:nx:end,:);


% constraints = [constraints , x1  <=  xmax_bold(nx+1:nx:end), ...
%     - x1 <= - xmin_bold(nx+1:nx:end), ...
%     x2  <=  xmax_bold(nx+2:nx:end) , ...
%     - x2 <= - xmin_bold(nx+2:nx:end)];



%%%%%%% 3rd and 4th states have COUPLED CONSTRAINTS: ...
%       x4 - c1*x3 <= - c3
%       x4 - c1*x3 >= - c2
%       x4 + c1*x3 <= c2
%       x4 + c1*x3 >= c3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

constraints = [constraints , x4 - c1*x3 <=  - repmat(c3, T, 1) , ...
    - x4 + c1*x3  <=   repmat(c2, T, 1),...
    x4 + c1*x3  <=   repmat(c2, T, 1)
    - x4 - c1*x3  <=  - repmat(c3, T, 1) ];


%%% Inputs have COUPLED CONSTRAINTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       u2 - t1*u1 <= - t3
%       u2 - t1*u1 >= - t2
%       u2 + t1*u1 <= t2
%       u2 + t1*u1 >= t3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
u1 = u(1:nu:end);
u2 = u(2:nu:end);

constraints = [constraints , u2 - t1*u1  <=  - repmat(t3, T, 1) , ...
    - u2 + t1*u1  <=   repmat(t2, T, 1),...
    u2 + t1*u1  <=   repmat(t2, T, 1)
    - u2 - t1*u1  <=  - repmat(t3, T, 1) ];


%%% Rotate the coordinate frame
Rot = EV.Rot;
pos_rot = Rot * [x1'; x2'];
x1_rot = pos_rot(1,:)' + EV.p0(1);
x2_rot = pos_rot(2,:)' + EV.p0(2);

% vel_rot = Rot * [x3'; x4'];
% x3_rot = vel_rot(1,:)';
% x4_rot = vel_rot(2,:)';

% for t = 1:T
%     for clu = 1:K
%         A_obs = A_union{clu,t};
%         b_obs = b_union{clu,t};
%         
%         if clu == 1 % truck going straight
%         constraints_obs = [constraints_obs , ...
%             A_obs * [x1_rot(t); x2_rot(t)] + big_M*(1-delta(4*(clu-1)+(1:4),t)) >= b_obs + repmat(car.d(2)/2 * 1.1, size(b_obs)), ...
%             sum(delta(4*(clu-1)+(1:4),t)) >= 1];
%         elseif clu == 2 % truck turning
%         constraints_obs = [constraints_obs , ...
%             A_obs * [x1_rot(t); x2_rot(t)] + big_M*(1-delta(4*(clu-1)+(1:4),t)) >= b_obs + repmat(diag, size(b_obs)), ...
%             sum(delta(4*(clu-1)+(1:4),t)) >= 1];
%         end    
%         
%     end
% end


for i=1:num_OV
    num_lat = OV{i}.num_latent;
    sum_clu = sum(K(1:i-1));
    for j=1:num_lat
        for t=1:T
            A_obs = A_union{t,j,i};
            b_obs = b_union{t,j,i};
            
            constraints_obs = [constraints_obs , ...
            A_obs * [x1_rot(t); x2_rot(t)] + big_M*(1-delta(4*(sum_clu+j-1)+(1:4),t)) >= b_obs + repmat(diag, size(b_obs)), ...
            sum(delta(4*(sum_clu+j-1)+(1:4),t)) >= 1];
        end
    end
end


Constraints = [constraints, constraints_obs];


%%% objective
objective = x2(end)^2 + x3(end)^2 + x4(end)^2 - x1(end);

%%% Solve the problem
ops = sdpsettings;
ops.solver = 'cplex';
ops.verbose = 3;
ops.debug = 1;

DIAGNOSTIC = optimize(Constraints, objective, ops)

%% Save results
u_star = value(u);
cost = value(objective);


x1_v = value(x1);
x2_v = value(x2);
x_star = Rot * [x1_v'; x2_v'] + EV.p0';
x_star = [EV.p0' x_star];
% plot(pos_rot(1,:), pos_rot(2,:), 'bo', 'MarkerFaceColor', 'b')
end

