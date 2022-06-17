function [rate_viol, viol_amt] = montecarlo_newpred(params, x_star)
%% Load needed parameters
car.d = params.car.d ;
CAR_R = sqrt(car.d(1)^2 + car.d(2)^2);
truck.d = params.truck.d ;
truck.theta0 = params.truck.theta0;
truck.start = params.truck.start ;
T = params.T;

%% Generate or load monte carlo data
% [position_OV, yaw_OV] = generate_monteCarlo(scene)
load('./MonteCarlo_data/monteCarlo_params_2.mat'); % Other monteCarlo data with various sizes are available

%% Run Monte Carlo tests
num_samples = length(position_OV);
disp(['Number of new predictions used: ', num2str(num_samples)]);

%%% Rotate the coordinate frame
x1_rot = x_star(1,:);
x2_rot = x_star(2,:);

num_viol = 0;
viol_amt = 0;

for i=1:num_samples
    if mod (i, 10000) == 0
        disp(['Progress: ', num2str(i), '/', num2str(num_samples)])
    end
    
    for t=1:T
        coeff1 = [cos(yaw_OV(i,t)), -sin(yaw_OV(i,t)), -cos(yaw_OV(i,t)) .* position_OV(i,t,1) + sin(yaw_OV(i,t)) .* position_OV(i,t,2) - truck.d(2)/2];
        coeff2 = [sin(yaw_OV(i,t)), cos(yaw_OV(i,t)), -sin(yaw_OV(i,t)) .* position_OV(i,t,1) - cos(yaw_OV(i,t)) .* position_OV(i,t,2) - truck.d(1)/2];
        coeff3 = [-cos(yaw_OV(i,t)), sin(yaw_OV(i,t)), cos(yaw_OV(i,t)) .* position_OV(i,t,1) - sin(yaw_OV(i,t)) .* position_OV(i,t,2) - truck.d(2)/2];
        coeff4 = [-sin(yaw_OV(i,t)), -cos(yaw_OV(i,t)), sin(yaw_OV(i,t)) .* position_OV(i,t,1) + cos(yaw_OV(i,t)) .* position_OV(i,t,2) - truck.d(1)/2];
        temp_x = [x1_rot(t+1), x2_rot(t+1), 1];
        
        if  1/2 * CAR_R >= coeff1 * temp_x' && 1/2 * CAR_R >= coeff2 * temp_x' && 1/2 * CAR_R >= coeff3 * temp_x' && 1/2 * CAR_R >= coeff4 * temp_x'
            num_viol = num_viol + 1;
            viol_amt = viol_amt + mean([coeff1/norm(coeff1) * temp_x', coeff2/norm(coeff2) * temp_x', coeff3/norm(coeff3) * temp_x', coeff4/norm(coeff4) * temp_x']);
            break
        end
    end
end

rate_viol = (num_viol/num_samples)*100;
viol_amt = viol_amt/num_viol;
end    

