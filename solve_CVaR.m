function [ u_star, cost, x_star, DIAGNOSTIC ] = solve_CVaR(params, OV, EV, eps_assign)
%%  Load needed parameters
car.d = params.car.d ;
CAR_R = 1/2 * sqrt(car.d(1)^2 + car.d(2)^2);
truck.d = params.truck.d ;
truck.theta0 = params.truck.theta0;
truck.start = params.truck.start ;
T = params.T;
O = params.O;
L = params.L;
nx = params.nx;
nu = params.nu;
Gamma = params.Gamma ;
States_free_init = params.States_free_init;
c1 = params.c1;
c2 = params.c2;
c3 = params.c3;
t1 = params.t1;
t2 = params.t2;
t3 = params.t3;

%% Optimization variables

u = sdpvar(nu*T,1,'full');

% binary variables for obstacle
delta = binvar(L,T,O,length(eps_assign));
big_M = 200; % conservative upper-bound

%% Constraints
constraints = [];

x_bold = sdpvar(nx*T,1,'full');
x_future = States_free_init + Gamma * u;

% state box constraints
constraints = [constraints,...
    x_bold == x_future(nx+1:end,:)];

x1 = x_bold(1:nx:end,:);
x2 = x_bold(2:nx:end,:);
x3 = x_bold(3:nx:end,:);
x4 = x_bold(4:nx:end,:);

constraints = [constraints , x4 - c1*x3  <=  - repmat(c3, T, 1) , ...
    - x4 + c1*x3  <=   repmat(c2, T, 1),...
    x4 + c1*x3  <=   repmat(c2, T, 1)
    - x4 - c1*x3  <=  - repmat(c3, T, 1) ];

%% Inputs have COUPLED CONSTRAINTS:
u1_bold = u(1:nu:end,1);
u2_bold = u(2:nu:end,1);

constraints = [constraints , u2_bold - t1*u1_bold  <=  - repmat(t3, T, 1) , ...
    - u2_bold + t1*u1_bold  <=   repmat(t2, T, 1),...
    u2_bold + t1*u1_bold  <=   repmat(t2, T, 1)
    - u2_bold - t1*u1_bold  <=  - repmat(t3, T, 1) ];

%%% Rotate the coordinate frame
Rot = EV.Rot;
pos_rot = Rot * [x1'; x2'];
x1_rot = pos_rot(1,:)' + EV.p0(1);
x2_rot = pos_rot(2,:)' + EV.p0(2);

constraints_obs = [];
num_OV = length(OV);

nd = 1;

for i=1:num_OV
    num_lat = OV{i}.num_latent;
    for j=1:num_lat
        for t=1:T
            beta_r1 = 1e-3 / 2;
            beta_r2 = 1e-3 / (2*nd);
            
            yawData = vertcat(OV{i}.pred_yaw{j});
            poseData = vertcat(OV{i}.pred_position{j});

            coeff1 = [cos(yawData(:,t)), -sin(yawData(:,t)), -cos(yawData(:,t)) .* poseData(:,t,1) + sin(yawData(:,t)) .* poseData(:,t,2) - truck.d(2)/2];
            coeff2 = [sin(yawData(:,t)), cos(yawData(:,t)), -sin(yawData(:,t)) .* poseData(:,t,1) - cos(yawData(:,t)) .* poseData(:,t,2) - truck.d(1)/2];
            coeff3 = [-cos(yawData(:,t)), sin(yawData(:,t)), cos(yawData(:,t)) .* poseData(:,t,1) - sin(yawData(:,t)) .* poseData(:,t,2) - truck.d(2)/2];
            coeff4 = [-sin(yawData(:,t)), -cos(yawData(:,t)), sin(yawData(:,t)) .* poseData(:,t,1) + cos(yawData(:,t)) .* poseData(:,t,2) - truck.d(1)/2];
            temp_x = [x1_rot(t) ; x2_rot(t) ; 1];
                
            eps_ijt = eps_assign(i, j)/T;

            constraints_obs = [constraints_obs , ...      
                mean(-coeff1 * temp_x + CAR_R) + sqrt(var(-coeff1 * temp_x + CAR_R))*((normpdf(norminv(1-eps_ijt)))/eps_ijt) - big_M*(1-delta(1,t,i,j)') <= 0 ,...
                mean(-coeff2 * temp_x + CAR_R) + sqrt(var(-coeff2 * temp_x + CAR_R))*((normpdf(norminv(1-eps_ijt)))/eps_ijt) - big_M*(1-delta(2,t,i,j)') <= 0 ,...
                mean(-coeff3 * temp_x + CAR_R) + sqrt(var(-coeff3 * temp_x + CAR_R))*((normpdf(norminv(1-eps_ijt)))/eps_ijt) - big_M*(1-delta(3,t,i,j)') <= 0 ,...
                mean(-coeff4 * temp_x + CAR_R) + sqrt(var(-coeff4 * temp_x + CAR_R))*((normpdf(norminv(1-eps_ijt)))/eps_ijt) - big_M*(1-delta(4,t,i,j)') <= 0 ,...
                ];

            constraints_obs = [constraints_obs, ...
                sum(delta(:,t,i,j)) == 1];
        end
    end
end

Constraints = [constraints, constraints_obs];


%% objective
objective =  x2(end)^2 + x3(end)^2 + x4(end)^2 - (x1(end));

%% Solve the problem
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

