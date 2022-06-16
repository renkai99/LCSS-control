function [position_OV, yaw_OV] = generate_monteCarlo(scene)
%% Generate Monte Carlo data
if scene == 2
    fileName = './data/predict_scene556_t6_data/predict_scene556_t6_meta_7000.json'; % filename in JSON extension
elseif scene == 1
    fileName = './data/predict_scene105_t11_data/predict_scene105_t11_meta_50000.json'; % filename in JSON extension
    fileName2 = './data/predict_scene105_t11_data/predict_scene105_t11_meta_50000_2.json'; % filename in JSON extension
    fileName3 = './data/predict_scene105_t11_data/predict_scene105_t11_meta_50000_3.json'; % filename in JSON extension
    fileName4 = './data/predict_scene105_t11_data/predict_scene105_t11_meta_50000_4.json'; % filename in JSON extension
end

str = fileread(fileName); % dedicated for reading files as text
str2 = fileread(fileName2); % dedicated for reading files as text
str3 = fileread(fileName3); % dedicated for reading files as text
str4 = fileread(fileName4); % dedicated for reading files as text
forecast1 = jsondecode(str);
forecast2 = jsondecode(str2);
forecast3 = jsondecode(str3);
forecast4 = jsondecode(str4);
forecast1.sc = scene;
forecast2.sc = scene;
forecast3.sc = scene;
forecast4.sc = scene;
[~, OV1] = process_trajectron_data(forecast1);
[~, OV2] = process_trajectron_data(forecast2);
[~, OV3] = process_trajectron_data(forecast3);
[~, OV4] = process_trajectron_data(forecast4);

position_OV = [OV1{1}.pred_position{1}; OV2{1}.pred_position{1}; OV3{1}.pred_position{1}; OV4{1}.pred_position{1}];
yaw_OV = [OV1{1}.pred_yaw{1}; OV2{1}.pred_yaw{1}; OV3{1}.pred_yaw{1}; OV4{1}.pred_yaw{1}];
for i=1:OV1{2}.num_latent
    position_OV = [position_OV; OV1{2}.pred_position{i}; OV2{2}.pred_position{i}; OV3{2}.pred_position{i}; OV4{2}.pred_position{i}];
    yaw_OV = [yaw_OV; OV1{2}.pred_yaw{i}; OV2{2}.pred_yaw{i}; OV3{2}.pred_yaw{i}; OV4{2}.pred_yaw{i}];
end
end