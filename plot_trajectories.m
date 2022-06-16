%% Plots for paper
[w_vertex] = get_vertex(params, OV);
fontsize = 22;
ind_frames = [1 3 6 9];
N_frames = length(ind_frames);

h = figure();
set(h, 'Position', [100, 100, 700, 600], 'Color', 'w')

color_OV = [0 0.5 0; 1 0 0 ; 0.929 0.694 0.125; 0.494 0.184 0.556];
for i = 1 : N_frames
    ind = ind_frames(i);
    hs = subplot(2,2,i); 
    hold on; axis equal;
    
    % place the map image
    if scene == 1
         map = imread('./data/predict_scene105_t11_data/predict_scene105_t11_map.png', 'png', 'BackgroundColor', 'none');
    elseif scene == 2   
        map = imread('./data/predict_scene556_t6_data/predict_scene556_t6_map.png', 'png', 'BackgroundColor', 'none');
    end
    
    im = image(flipud(map), 'XData', [forecast.patch_extent(1)-20, forecast.patch_extent(3)+20], ...
        'YData', [forecast.patch_extent(2)-10, forecast.patch_extent(4)+10]);
    im.AlphaData = 0.3;
    title(['$$t=$$', num2str((ind-1))], 'interpreter', 'latex')
    
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
    
    % Past OV trajectories and current OV pose
    if ind == 1
        for o=1:O
            x0_truck = [OV{o}.p0'; OV{o}.yaw0];
            plot(x0_truck(1), x0_truck(2), 'o', 'MarkerEdgeColor', color_OV(o,:))
            plot(OV{o}.past(:,1), OV{o}.past(:,2), '-d', 'MarkerEdgeColor', color_OV(o,:), 'LineWidth', 1);
            v_truck = generate_vertex(params, 'truck', x0_truck);
            p_truck = polyshape(v_truck.x, v_truck.y);
            plot(p_truck, 'EdgeColor', color_OV(o,:), 'FaceColor', 'none', 'LineWidth', 2)
        end
    else
        for o = 1:O
            % Future OV state distributions
            for clu = 1:OV{o}.num_latent
                color_lat = 0.9*color_OV(o,:) + 1.25*(clu-1)*([1 1 1]-color_OV(o,:))/OV{o}.num_latent;
                vertexes_plot = w_vertex{ind-1, clu, o};
                plot(vertexes_plot(1,:), vertexes_plot(2,:), '.', 'MarkerEdgeColor', color_lat)
                plot(vertexes_plot(3,:), vertexes_plot(4,:), '.', 'MarkerEdgeColor', color_lat)
            end
        end
    end
    
    if scene == 1
        axis([855 900 1395 1435])
    elseif scene == 2
        axis([1195 1245 950 985])
    end
   
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
    
    if i == N_frames
        legend({'MTA','MRA','CVaR','CVaRR'}, 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 12);
        Lgnd = legend('show');
        Lgnd.Position(1) = 0.23;
        Lgnd.Position(2) = 0.5;
        
        % MagInset for two scenes
        if scene == 1
            MagInset(h, hs, [864.5 870 1421 1426], [885 895 1397 1407], {'SW','NW';'SE','NE'}); grid off; legend('hide'); set(gca,'xtick',[]); set(gca,'ytick',[]); set(gca,'XTickLabel',[]); set(gca,'YTickLabel',[]);
        elseif scene == 2
            MagInset(h, hs, [1215.8 1220 958.3 961.5], [1198 1211 970 980], {'NW','SW';'NE','SE'}); grid off; legend('hide'); set(gca,'xtick',[]); set(gca,'ytick',[]); set(gca,'XTickLabel',[]); set(gca,'YTickLabel',[]);
        end
    end
end