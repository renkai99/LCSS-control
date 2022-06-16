%% script for simulation results in CDC 2021
clear all
close all

%% load trajectron++ forecast results
scene = 1;

if scene == 1
    fileName = './data/predict_scene105_t11_data/predict_scene105_t11_meta_7000.json'; % filename in JSON extension
elseif scene == 2
    fileName = './data/predict_scene556_t6_data/predict_scene556_t6_meta_7000.json'; % filename in JSON extension
end
str = fileread(fileName);
forecast = jsondecode(str);
forecast.sc = scene;
[EV, OV] = process_trajectron_data(forecast);

%% load parameters
run problem_parameters

%% Moment Trust Approach 
fprintf(['-------------------------------------------------------------------------\n',...
        'Solve the MTA problem ...\n']);
[u_MTA, cost_MTA, car_states_MTA, DIAGNOSTIC_MTA] = solve_MTA(params, OV, EV, eps_ura);
compTime_MTA = DIAGNOSTIC_MTA.solvertime;
fprintf('Completed.\n');

%% Moment Robust Approach 
fprintf(['-------------------------------------------------------------------------\n',...
        'Solve the MRA problem ...\n']);
[u_MRA, cost_MRA, car_states_MRA, DIAGNOSTIC_MRA] = solve_MRA(params, OV, EV, eps_ura);
compTime_MRA = DIAGNOSTIC_MRA.solvertime;
fprintf('Completed.\n');

%% CVaR 
fprintf(['-------------------------------------------------------------------------\n',...
        'Solve the CVaR problem ...\n']);
[u_CVaR, cost_CVaR, car_states_CVaR, DIAGNOSTIC_CVaR] = solve_CVaR(params, OV, EV, eps_ura);
compTime_CVaR = DIAGNOSTIC_CVaR.solvertime;
fprintf('Completed.\n');

%% CVaRR 
fprintf(['-------------------------------------------------------------------------\n',...
        'Solve the CVaRR problem ...\n']);
[u_CVaRR, cost_CVaRR, car_states_CVaRR, DIAGNOSTIC_CVaRR] = solve_CVaRR(params, OV, EV, eps_ura);
compTime_CVaRR = DIAGNOSTIC_CVaRR.solvertime;
fprintf('Completed.\n');

%% Scenario Bounding Box
fprintf(['-------------------------------------------------------------------------\n',...
        'Solve the Scenario Bounding Box method ...\n']);
[u_scenarioBox, cost_scenarioBox, car_states_scenarioBox, A_union, b_union, t_overapprox, vertex, DIAGNOSTIC_scenarioBox] = solve_scenarioBox(params, OV, EV);
compTime_scenarioBox = DIAGNOSTIC_scenarioBox.solvertime;

%% Plot for the paper
run plot_trajectories

%% Emperical rate and amount of violations (Monte Carlo)
% [emp_rate_viol_MTA, emp_viol_amt_MTA] = montecarlo_viol(params, OV, car_states_MTA);
% [emp_rate_viol_MRA, emp_viol_amt_MRA] = montecarlo_viol(params, OV, car_states_MRA);
% [emp_rate_viol_CVaR, emp_viol_amt_CVaR] = montecarlo_viol(params, OV, car_states_CVaR);
% [emp_rate_viol_CVaRR, emp_viol_amt_CVaRR] = montecarlo_viol(params, OV, car_states_CVaRR);
% [emp_rate_viol_scenarioBox, emp_viol_amt_scenarioBox] = montecarlo_viol(params, OV, car_states_scenarioBox);

%% Worst case collisions
% [worst_coeff_MTA, worst_param_MTA] = montecarlo_worstCase(params, OV, car_states_MTA);
% [worst_coeff_MRA, worst_param_MRA] = montecarlo_worstCase(params, OV, car_states_MRA);
% [worst_coeff_CVaR, worst_param_CVaR] = montecarlo_worstCase(params, OV, car_states_CVaR);
% [worst_coeff_CVaRR, worst_param_CVaRR] = montecarlo_worstCase(params, OV, car_states_CVaR);

%% Monte_carlo with new predictions
% [rate_viol_MTA, viol_amt_MTA] = montecarlo_newpred(params, car_states_MTA);
% [rate_viol_MRA, viol_amt_MRA] = montecarlo_newpred(params, car_states_MRA);
% [rate_viol_CVaR, viol_amt_CVaR] = montecarlo_newpred(params, car_states_CVaR);
% [rate_viol_CVaRR, viol_amt_CVaRR] = montecarlo_newpred(params, car_states_CVaRR);
