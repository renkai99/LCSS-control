function [worst_coeff, worst_param] = montecarlo_worstCase(params, OV, x_star)
    %%  Load needed parameters
    car.d = params.car.d ;
    CAR_R = sqrt(car.d(1)^2 + car.d(2)^2);
    truck.d = params.truck.d ;
    truck.theta0 = params.truck.theta0;
    truck.start = params.truck.start ;
    T = params.T;
    num_OV = length(OV);

    %%% Rotate the coordinate frame
    x1_rot = x_star(1,:);
    x2_rot = x_star(2,:);

    num_mc = 1e4;
    worst_amt = 0;
    worst_coeff = [0, 0, 0];

    for i=1:num_OV
        num_lat = OV{i}.num_latent;
        for j=1:num_lat
            for t=1:T
                disp([i, j, t])
                pose_data = [OV{i}.pred_position{j}(:, t, 1), OV{i}.pred_position{j}(:, t, 2), OV{i}.pred_yaw{j}(:, t)];
                pose_avg = mean(pose_data);
                pose_cov = cov(pose_data);
                for k = 1:num_mc
                    rng(k)
                    random_coeff = randn(1,3);
                    pose_current = repmat(pose_avg,1,1) + 2*random_coeff*chol(pose_cov);
                    
                    coeff1 = [cos(pose_current(1, 3)), -sin(pose_current(1, 3)), -cos(pose_current(1, 3)) .* pose_current(1, 1) + sin(pose_current(1, 3)) .* pose_current(1, 2) - truck.d(2)/2];
                    coeff2 = [sin(pose_current(1, 3)), cos(pose_current(1, 3)), -sin(pose_current(1, 3)) .* pose_current(1, 1) - cos(pose_current(1, 3)) .* pose_current(1, 2) - truck.d(1)/2];
                    coeff3 = [-cos(pose_current(1, 3)), sin(pose_current(1, 3)), cos(pose_current(1, 3)) .* pose_current(1, 1) - sin(pose_current(1, 3)) .* pose_current(1, 2) - truck.d(2)/2];
                    coeff4 = [-sin(pose_current(1, 3)), -cos(pose_current(1, 3)), sin(pose_current(1, 3)) .* pose_current(1, 1) + cos(pose_current(1, 3)) .* pose_current(1, 2) - truck.d(1)/2];                    
                    temp_x = [x1_rot(t+1), x2_rot(t+1), 1];

                    if  1/2 * CAR_R >= coeff1 * temp_x' && 1/2 * CAR_R >= coeff2 * temp_x' && 1/2 * CAR_R >= coeff3 * temp_x' && 1/2 * CAR_R >= coeff4 * temp_x'
                        viol_amout = mean([coeff1 * temp_x', coeff2 * temp_x', coeff3 * temp_x', coeff4 * temp_x']);
                        if viol_amout < worst_amt
                            worst_amt = viol_amout;
                            worst_coeff = pose_current;
                            worst_param = [i, j, t];
                        end
                        break
                    end
                end
            end
        end
    end
end