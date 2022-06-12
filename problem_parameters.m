%% This Script loads the needed problem parameters

% System dynamics
Ts = 0.5;
sys = c2d(ss([zeros(2) eye(2); zeros(2,4)], [zeros(2); eye(2)],...
    [eye(2) zeros(2)], zeros(2)),Ts); % Double integrator model
A = sys.A;
B = sys.B;
[nx,nu] = size(B);

%% Problem parameters
lane = 3.5;

% Car and truck dimensions
car.d = [4.5; 2.5];
diag = sqrt(car.d(2)^2 + car.d(1)^2)/2;
car.v = 10/3.6; % initial forward velocity in m/s

truck.d = [4.5; 2.5];
%truck.v = -20/3.6;  %truck initial velocity
truck.theta0 = 0 ; % truck initial orientation
truck.start = [42; lane/2];

% Road dimensions
xmin = [0; -lane]; % lower bounds
xmax = [100+4*car.d(1); 0]; % upper bounds

% Vehicle dynamics restrictions
vmin = [0/3.6 ; -20/3.6]; % lower bounds
vmax = [80/3.6 ; 20/3.6] ; % upper bounds
umin = [-10; -5];
umax = [3; 5];

T = 8; % prediction horizon
O = length(OV); % number of obstacles
L = 4; % number of faces of obstacle sets
K = zeros(1,O); % number of clusters 
for i=1:O
    K(i) = OV{i}.num_latent;
end
middle = 0; % vertical center of the road
x0 = [ 0 0 EV.v0 0]'; % initial state
%% Disturbance
nw = 3; % Truck's position and orientation

%% State and input constraints (Box constraints)
% xmin_bold = repmat([xmin + car.d/2; vmin], T+1,1);
% xmax_bold = repmat([xmax - car.d/2; vmax], T+1,1);
umin_bold = repmat( umin, T,1);
umax_bold = repmat( umax, T,1);

%% 3rd and 4th states have COUPLED CONSTRAINTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       x4 - c1*x3 <= - c3
%       x4 - c1*x3 >= - c2
%       x4 + c1*x3 <= c2
%       x4 + c1*x3 >= c3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coefficients for the coupled constraints for 3rd and 4th states
c1 = vmax(2)/(0.5*(vmax(1)-vmin(1)));  % they work only assuming  vmax(2) = -vmin(2)
c2 = c1*vmax(1);
c3 = c1*vmin(1);

%% Inputs have COUPLED CONSTRAINTS:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       u2 - t1*u1 <= - t3
%       u2 - t1*u1 >= - t2
%       u2 + t1*u1 <= t2
%       u2 + t1*u1 >= t3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coefficients for the coupled constraints for 3rd and 4th states
t1 = umax(2)/(0.5*(umax(1)-umin(1)));  % they work only assuming  vmax(2) = -vmin(2)
t2 = t1*umax(1);
t3 = t1*umin(1);

%% 
Abar = eye(T * nx) - [zeros(nx, T * nx); kron(eye(T-1),A) zeros((T-1)*nx,nx)];
Bbar = kron(eye(T),B);
Gamma = [zeros(nx, T*nu); Abar \ Bbar]; % Abar*Gamma = Bbar except for first rows

States_free_init  = [];
for k = 1:T+1
    States_free_init = [ States_free_init ; (A^(k-1))*x0];
end

%% decide number of samples and risk allocation
eps = 0.05;
beta = 1e-3;

N_scenario = ceil(1.59/eps * (log(2^(L*T*O)/beta) + T*nu - 1));
N_proposed = zeros(O, max(K));
eps_ura = zeros(O, max(K));
eps_assign = zeros(O, max(K));
beta_assign = zeros(O, max(K));

if scene == 3
    i = 1;
        eps1 = eps/9.14;
        beta1 = beta/9.14;
        eps_assign(i,1) = eps1;
        beta_assign(i,1) = beta1;
        N_proposed(i,1) = ceil(1.59/eps1 * (log(1/beta1) + L*T - 1));
        
    i = 2;
    eps_res = eps - eps1;
    beta_res = eps - beta1;
    for k=1:K(i)
        eps_assign(i,k) = eps_res * 1/OV{i}.latent_pmf(k)/sum(1./OV{i}.latent_pmf);
        beta_assign(i,k) = beta_res * 1/OV{i}.latent_pmf(k)/sum(1./OV{i}.latent_pmf);
        N_proposed(i,k) = ceil(1.59/eps_assign(i,k) * (log(1/beta_assign(i,k)) + L*T - 1));
    end
else
    for i=1:O
        for k=1:K(i)
            eps_assign(i,k) = eps/O * 1/OV{i}.latent_pmf(k)/sum(1./OV{i}.latent_pmf);
            beta_assign(i,k) = beta/O * 1/OV{i}.latent_pmf(k)/sum(1./OV{i}.latent_pmf);
            N_proposed(i,k) = ceil(1.59/eps_assign(i,k) * (log(1/beta_assign(i,k)) + L*T - 1));
        end
    end
end

for i=1:O
    for k=1:K(i)
        eps_ura(i,k) = eps/(O);
    end
end

weights = normr(eps_assign);
weights(2, :) = weights(2, :)/sum(weights(2, :));

%% Set problem parameters
params.lane = lane;
params.car.d  = car.d;
params.car.v = car.v;
params.x0 = x0;
params.truck.d = truck.d;
params.truck.theta0 = truck.theta0;
params.truck.start = truck.start;
params.xmin = xmin;
params.xmax = xmax;
params.vmin = vmin;
params.vmax = vmax;
params.umin = umin;
params.umax = umax;
params.T = T;
params.middle = middle;
params.nx = nx;
params.nu = nu;
params.nw = nw;
params.A = A;
params.B = B;
params.Gamma = Gamma;
params.States_free_init = States_free_init;
params.c1 = c1;
params.c2 = c2;
params.c3 = c3;
params.t1 = t1;
params.t2 = t2;
params.t3 = t3;
params.diag = diag;
params.Ts = Ts;
params.O = O;
params.L = L;
params.K = K;
params.N_scenario = N_scenario;
params.N_proposed = N_proposed;

params.EV = EV;
params.OV = OV;

