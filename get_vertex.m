function [vertex] = get_vertex(params, OV)
%%  Load needed parameters
truck.d = params.truck.d ;
truck.theta0 = params.truck.theta0;
truck.start = params.truck.start ;
T = params.T;
K = params.K;

%% Find vertexes of sampled obstacle sets
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

                vertex{t,j,i}(1:2,k) = [truck_y1; truck_y2] + 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [-sin(truck_theta); cos(truck_theta)];
                vertex{t,j,i}(3:4,k) = [truck_y1; truck_y2] + 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [sin(truck_theta); -cos(truck_theta)];
                vertex{t,j,i}(5:6,k) = [truck_y1; truck_y2] - 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [-sin(truck_theta); cos(truck_theta)];
                vertex{t,j,i}(7:8,k) = [truck_y1; truck_y2] - 1/2 * truck.d(1) * [cos(truck_theta); sin(truck_theta)] + 1/2 * truck.d(2) * [sin(truck_theta); -cos(truck_theta)];
            end
        end
    end
end
end