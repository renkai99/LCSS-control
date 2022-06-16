function [EV, OV] = process_trajectron_data(forecast)
T = forecast.prediction_horizon;
dt = 0.5;
n_veh = forecast.n_vehicles;

total_sample = 0;
for i=1:25
    total_sample = total_sample + size(forecast.vehicles(1).latent_predictions(i).predictions,1);
end
scene = forecast.sc;
    
if scene == 1
    X = 1.0e+03 * [    0.8592    1.4310;
    0.8521    1.4375;
    0.8466    1.4425;
    0.8408    1.4478];
    EV.p0 = X(1,:);
    EV.v0 = 0;
    EV.yaw0 = atan2(X(1,2)-X(2,2), X(1,1)-X(2,1));
    EV.Rot = [cos(EV.yaw0), -sin(EV.yaw0); sin(EV.yaw0) cos(EV.yaw0)];
    EV.past = X;
elseif scene == 2
    X = 1.0e+03 * [1.2127    0.9539;
    1.2052    0.9448;
    1.1984    0.9368;
    1.1884    0.9248];
    EV.p0 = X(1,:);
    EV.v0 = 0;
    EV.yaw0 = atan2(X(1,2)-X(2,2), X(1,1)-X(2,1));
    EV.Rot = [cos(EV.yaw0), -sin(EV.yaw0); sin(EV.yaw0) cos(EV.yaw0)];
    EV.past = X;
end

if scene == 1
    v_interest = [3, 7]; % (t10, 11)3 7 (t12, 13, 14, 15) 3 6
elseif scene == 2
    v_interest = [2, 7]; % for scene556_t6 (2 7) 7 8 (2 6) 9 10 11 12 (2 5)
end
n_interest = length(v_interest);
v_color = ['k', 'r', 'g', 'y', 'c', 'm'];

for j = 1:n_interest
    veh = v_interest(j);
    data_veh = forecast.vehicles(veh);
    truth = data_veh.ground_truth;
    past = data_veh.past;
    
    position_last = past(end,:);
    position_prev = past(end-3,:);
    yaw = atan2(position_last(2)-position_prev(2), position_last(1) - position_prev(1));
    OV{j}.p0 = position_last;
    OV{j}.yaw0 = yaw;
    OV{j}.past = past;
    OV{j}.truth = truth;

    if scene == 2
        set_lat = find(data_veh.latent_pmf > 0.08)
    else
        set_lat = find(data_veh.latent_pmf > 0.1) %0.07 for 14, 15
    end
    
    OV{j}.num_latent = length(set_lat);
    
    pred_position = {};
    pred_yaw = {};
    k_mean = zeros(2,length(set_lat));
    for i=1:length(set_lat)
        pred = data_veh.latent_predictions(set_lat(i)).predictions;
        yaw = zeros(length(pred),1);
        diff = (pred(:,1,2)-position_last(2)).^2 + (pred(:,1,1)-position_last(1)).^2;
        i_keep_yaw = find(diff < 0.5);
        yaw(:,1) = atan2(pred(:,1,2)-position_last(2), pred(:,1,1)-position_last(1));
        yaw(i_keep_yaw,1) = OV{j}.yaw0;
        
        
        for t=2:T
            diff = (pred(:,t,2)-pred(:,t-1,2)).^2 + (pred(:,t,1)-pred(:,t-1,1)).^2;
            i_keep_yaw = find(diff < 0.5);
            yaw(:,t) = atan2(pred(:,t,2)-pred(:,t-1,2), pred(:,t,1)-pred(:,t-1,1));
            yaw(i_keep_yaw,t) = yaw(i_keep_yaw,t-1);
        end
%         plot(pred(:,:,1), pred(:,:,2), '.', 'MarkerEdgeColor', v_color(j+i-1))
        pred_position{i} = pred;
        pred_yaw{i} = yaw;
        
        k_mean(:,i) = [mean(pred(:,T,1)), mean(pred(:,T,2))]';
    end
    OV{j}.pred_position = pred_position;
    OV{j}.pred_yaw = pred_yaw;
    OV{j}.init_center = k_mean;
%     plot(truth(:,1), truth(:,2), 'w*')
    
    rare_lat = setdiff(1:25, set_lat);
    dist_center = zeros(1,length(set_lat));
    for i=1:length(rare_lat)
        pred = data_veh.latent_predictions(rare_lat(i)).predictions;
        if ~size(pred,1) == 0
            yaw = zeros(size(pred,1),1);
            diff = (pred(:,1,2)-position_last(2)).^2 + (pred(:,1,1)-position_last(1)).^2;
            i_keep_yaw = find(diff < 0.5);
            yaw(:,1) = atan2(pred(:,1,2)-position_last(2), pred(:,1,1)-position_last(1));
            yaw(i_keep_yaw,1) = OV{j}.yaw0;

            for t=2:T
                diff = (pred(:,t,2)-pred(:,t-1,2)).^2 + (pred(:,t,1)-pred(:,t-1,1)).^2;
                i_keep_yaw = find(diff < 0.5);
                yaw(:,t) = atan2(pred(:,t,2)-pred(:,t-1,2), pred(:,t,1)-pred(:,t-1,1));
                yaw(i_keep_yaw,t) = yaw(i_keep_yaw,t-1);
            end
            
%             disp(['car number', num2str(j)])
%             disp(set_lat)

            for k=1:size(pred,1)

                for clu = 1:length(set_lat)
                    dist_center(clu) = norm(k_mean(:,clu) - squeeze(pred(k,T,:)),2);
                end
                [~, in_clu] = min(dist_center);

                pred_position{in_clu} = [pred_position{in_clu}; pred(k, :, :)];
                pred_yaw{in_clu} = [pred_yaw{in_clu}; yaw(k, :)];
            end
        end
    end
    
    latent_pmf = zeros(1, length(set_lat));
    for i=1:length(set_lat)
        latent_pmf(i) = size(pred_position{i},1)/total_sample;
    end
    OV{j}.pred_position = pred_position;
    OV{j}.pred_yaw = pred_yaw;
    OV{j}.init_center = k_mean;
    OV{j}.latent_pmf = latent_pmf;
    
end




end