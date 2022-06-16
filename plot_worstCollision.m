%% Plots for paper
fontsize = 22;
color_OV = [0 0.5 0; 1 0 0 ; 0.929 0.694 0.125; 0.494 0.184 0.556];
ind = 9;

%% Scene 1
load('./Figures/worstParams_scene1.mat');
hs = subplot(1,2,1);
hold on; axis equal;
% place the map image
map = imread('./data/predict_scene105_t11_data/predict_scene105_t11_map.png');
im = image(flipud(map), 'XData', [forecast.patch_extent(1)-20, forecast.patch_extent(3)+20], ...
    'YData', [forecast.patch_extent(2)-10, forecast.patch_extent(4)+10]);
im.AlphaData = 0.3;

% EV trajectory
p3 = plot(car_states_MTA(1,1:ind), car_states_MTA(2,1:ind), 'g-^', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_MTA = generate_vertex(params, 'car', car_states_MTA(:,ind));
p2 = plot(car_states_MRA(1,1:ind), car_states_MRA(2,1:ind), 'r-o', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_MRA = generate_vertex(params, 'car', car_states_MRA(:,ind));
p1 = plot(car_states_CVaR(1,1:ind), car_states_CVaR(2,1:ind), 'b-*', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_CVaR = generate_vertex(params, 'car', car_states_CVaR(:,ind));
p4 = plot(car_states_CVaRR(1,1:ind), car_states_CVaRR(2,1:ind), 'm-*', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_CVaRR = generate_vertex(params, 'car', car_states_CVaRR(:,ind));

legend({'MTA','MRA','CVaR','CVaRR'}, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 12, 'AutoUpdate','off');
Lgnd = legend('show');
Lgnd.Position(1) = 0.19;
Lgnd.Position(2) = 0.17;

% EV polytope
p_car_MTA = polyshape(v_car_MTA.x, v_car_MTA.y);
plot(p_car_MTA,'EdgeColor', 'g', 'FaceColor', 'none', 'LineWidth', 1)
p_car_MRA = polyshape(v_car_MRA.x, v_car_MRA.y);
plot(p_car_MRA,'EdgeColor', 'r', 'FaceColor', 'none', 'LineWidth', 1)
p_car_CVaR = polyshape(v_car_CVaR.x, v_car_CVaR.y);
plot(p_car_CVaR,'EdgeColor', 'b', 'FaceColor', 'none', 'LineWidth', 1)
p_car_CVaRR = polyshape(v_car_CVaRR.x, v_car_CVaRR.y);
plot(p_car_CVaRR,'EdgeColor', 'm', 'FaceColor', 'none', 'LineWidth', 1)

% Worst case collision
for o = 1:O
    % Future OV state distributions
    for clu = 1:OV{o}.num_latent
        color_lat = 0.9*color_OV(o,:) + 1.25*(clu-1)*([1 1 1]-color_OV(o,:))/OV{o}.num_latent;
        if o == worst_param_MTA(1) && clu == worst_param_MTA(2) && (ind - 1) == worst_param_MTA(3)
            worst_truck = generate_vertex(params, 'truck', worst_coeff_MTA');
            p_worst = polyshape(worst_truck.x,worst_truck.y);
            plot(p_worst, 'EdgeColor', color_lat, 'FaceColor', color_lat, 'LineWidth', 1)
        end
    end
end

axis([858 872 1420 1432])
set(gca, 'box', 'on', 'Visible', 'on', 'LineWidth', 1)
set(gca, 'xticklabel',[], 'yticklabel',[])
set(gca,'TickLabelInterpreter', 'latex', 'FontSize', fontsize)
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
ax.FontSize = 12;
title('Scene 1')

%% Scene 2
load('Figures/worstParams_scene2.mat');
hs = subplot(1,2,2);
hold on; axis equal;
map = imread('./data/predict_scene556_t6_data/predict_scene556_t6_map.png', 'png', 'BackgroundColor', 'none');
im = image(flipud(map), 'XData', [forecast.patch_extent(1)-20, forecast.patch_extent(3)+20], ...
    'YData', [forecast.patch_extent(2)-10, forecast.patch_extent(4)+10]);
im.AlphaData = 0.3;

% EV trajectory
p3 = plot(car_states_MTA(1,1:ind), car_states_MTA(2,1:ind), 'g-^', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_MTA = generate_vertex(params, 'car', car_states_MTA(:,ind));
p2 = plot(car_states_MRA(1,1:ind), car_states_MRA(2,1:ind), 'r-o', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_MRA = generate_vertex(params, 'car', car_states_MRA(:,ind));
p1 = plot(car_states_CVaR(1,1:ind), car_states_CVaR(2,1:ind), 'b-*', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_CVaR = generate_vertex(params, 'car', car_states_CVaR(:,ind));
p4 = plot(car_states_CVaRR(1,1:ind), car_states_CVaRR(2,1:ind), 'm-*', 'MarkerSize', 4, 'LineWidth', 0.8);
v_car_CVaRR = generate_vertex(params, 'car', car_states_CVaRR(:,ind));

% EV polytope
p_car_MTA = polyshape(v_car_MTA.x, v_car_MTA.y);
plot(p_car_MTA,'EdgeColor', 'g', 'FaceColor', 'none', 'LineWidth', 1)
p_car_MRA = polyshape(v_car_MRA.x, v_car_MRA.y);
plot(p_car_MRA,'EdgeColor', 'r', 'FaceColor', 'none', 'LineWidth', 1)
p_car_CVaR = polyshape(v_car_CVaR.x, v_car_CVaR.y);
plot(p_car_CVaR,'EdgeColor', 'b', 'FaceColor', 'none', 'LineWidth', 1)
p_car_CVaRR = polyshape(v_car_CVaRR.x, v_car_CVaRR.y);
plot(p_car_CVaRR,'EdgeColor', 'm', 'FaceColor', 'none', 'LineWidth', 1)

% Worst case collision
for o = 1:O
    % Future OV state distributions
    for clu = 1:OV{o}.num_latent
        color_lat = 0.9*color_OV(o,:) + 1.25*(clu-1)*([1 1 1]-color_OV(o,:))/OV{o}.num_latent;
        if o == worst_param_MTA(1) && clu == worst_param_MTA(2) && (ind - 1) == worst_param_MTA(3)
            worst_truck = generate_vertex(params, 'truck', worst_coeff_MTA');
            p_worst = polyshape(worst_truck.x,worst_truck.y);
            plot(p_worst, 'EdgeColor', color_lat, 'FaceColor', color_lat, 'LineWidth', 1)
        end
    end
end

axis([1210 1224 952 963.5])
set(gca, 'box', 'on', 'Visible', 'on', 'LineWidth', 1)
set(gca, 'xticklabel',[], 'yticklabel',[])
set(gca,'TickLabelInterpreter', 'latex', 'FontSize', fontsize)
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
ax.FontSize = 12;
title('Scene 2')