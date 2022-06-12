function [rate_viol, viol_amt] = montecarlo_viol(params, OV, x_star, eps_assign)
    %%  Load needed parameters
    car.d = params.car.d ;
    CAR_LENGTH = car.d(1);
    CAR_WIDTH = car.d(2);
    truck.d = params.truck.d ;
    truck.theta0 = params.truck.theta0;
    truck.start = params.truck.start ;
    T = params.T;
    num_OV = length(OV);

    %%% Rotate the coordinate frame
    x1_rot = x_star(1,:);
    x2_rot = x_star(2,:);

    num_viol = zeros(num_OV, length(eps_assign));
    rate_viol = zeros(num_OV, length(eps_assign));
    viol_amt = zeros(num_OV, length(eps_assign));
    num_mc = 1e5;

    for i=1:num_OV
        num_lat = OV{i}.num_latent;
        for j=1:num_lat
            for t=1:T
                coeff1_avg = mean([cos(OV{i}.pred_yaw{j}(:,t)), -sin(OV{i}.pred_yaw{j}(:,t)), -cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) + sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(2)/2 ]);
                coeff2_avg = mean([sin(OV{i}.pred_yaw{j}(:,t)), cos(OV{i}.pred_yaw{j}(:,t)), -sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) - cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(1)/2 ]);
                coeff3_avg = mean([-cos(OV{i}.pred_yaw{j}(:,t)), sin(OV{i}.pred_yaw{j}(:,t)), cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) - sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(2)/2 ]);
                coeff4_avg = mean([-sin(OV{i}.pred_yaw{j}(:,t)), -cos(OV{i}.pred_yaw{j}(:,t)), sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) + cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(1)/2 ]);

                coeff1_cov = cov([cos(OV{i}.pred_yaw{j}(:,t)), -sin(OV{i}.pred_yaw{j}(:,t)), -cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) + sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(2)/2 ]);
                coeff2_cov = cov([sin(OV{i}.pred_yaw{j}(:,t)), cos(OV{i}.pred_yaw{j}(:,t)), -sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) - cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(1)/2 ]);
                coeff3_cov = cov([-cos(OV{i}.pred_yaw{j}(:,t)), sin(OV{i}.pred_yaw{j}(:,t)), cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) - sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(2)/2 ]);
                coeff4_cov = cov([-sin(OV{i}.pred_yaw{j}(:,t)), -cos(OV{i}.pred_yaw{j}(:,t)), sin(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,1) + cos(OV{i}.pred_yaw{j}(:,t)) .* OV{i}.pred_position{j}(:,t,2) - truck.d(1)/2 ]);
                
                for k = 1:num_mc
                    rng(k)
                    random_coeff = randn(1,3);
                    coeff1 = repmat(coeff1_avg,1,1) + random_coeff*chol(coeff1_cov);
                    coeff2 = repmat(coeff2_avg,1,1) + random_coeff*chol(coeff2_cov);
                    coeff3 = repmat(coeff3_avg,1,1) + random_coeff*chol(coeff3_cov);
                    coeff4 = repmat(coeff4_avg,1,1) + random_coeff*chol(coeff4_cov);
                    
                    for vertex = 1 : 5
                        if vertex == 1 
                            temp_x = [x1_rot(t+1)+CAR_LENGTH/2, x2_rot(t+1)+CAR_WIDTH/2, 1];
                        elseif vertex == 2
                            temp_x = [x1_rot(t+1)+CAR_LENGTH/2, x2_rot(t+1)-CAR_WIDTH/2, 1];
                        elseif vertex == 3
                            temp_x = [x1_rot(t+1)-CAR_LENGTH/2, x2_rot(t+1)+CAR_WIDTH/2, 1];
                        elseif vertex == 4
                            temp_x = [x1_rot(t+1)-CAR_LENGTH/2, x2_rot(t+1)-CAR_WIDTH/2, 1];
                        elseif vertex == 5
                            temp_x = [x1_rot(t+1), x2_rot(t+1), 1];
                        end

                        if  0 >= coeff1 * temp_x' && 0 >= coeff2 * temp_x' && 0 >= coeff3 * temp_x' && 0 >= coeff4 * temp_x'
                            num_viol(i, j) = num_viol(i, j) + 1;
                            viol_amt(i, j) = viol_amt(i, j) + mean([coeff1/norm(coeff1) * temp_x', coeff2/norm(coeff2) * temp_x', coeff3/norm(coeff3) * temp_x', coeff4/norm(coeff4) * temp_x']);
%                             viol_amt(i, j) = viol_amt(i, j) + max([coeff1 * temp_x', coeff2 * temp_x', coeff3 * temp_x', coeff4 * temp_x']);
                            break
                        end
                    end
                end
            end
            rate_viol(i, j) = num_viol(i, j)/num_mc;
            if num_viol(i, j) ~= 0
                viol_amt(i, j) = viol_amt(i, j)/(num_viol(i, j));
            end
        end
    end
    viol_amt = sum(sum(viol_amt));
    rate_viol = sum(sum(rate_viol))*100;
end
    


