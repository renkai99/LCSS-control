function vertex = generate_vertex(params, type, center)
    
    if isequal(type, 'truck')
        len = params.truck.d(1);
        wid = params.truck.d(2);
        yaw = center(3);
    else
        len = params.car.d(1);
        wid = params.car.d(2);
        yaw = params.EV.yaw0;
    end
       
    vertex1 = center(1:2,1) + 1/2 * len * [cos(yaw); sin(yaw)] + 1/2 * wid * [-sin(yaw); cos(yaw)];
    vertex2 = center(1:2,1) + 1/2 * len * [cos(yaw); sin(yaw)] + 1/2 * wid * [sin(yaw); -cos(yaw)];
    vertex3 = center(1:2,1) - 1/2 * len * [cos(yaw); sin(yaw)] + 1/2 * wid * [sin(yaw); -cos(yaw)];
    vertex4 = center(1:2,1) - 1/2 * len * [cos(yaw); sin(yaw)] + 1/2 * wid * [-sin(yaw); cos(yaw)];
    
    vertex.x = [vertex1(1), vertex2(1), vertex3(1), vertex4(1), vertex1(1)]';
    vertex.y = [vertex1(2), vertex2(2), vertex3(2), vertex4(2), vertex1(2)]';
    
end