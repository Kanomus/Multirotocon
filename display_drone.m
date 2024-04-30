function display_drone(position, system_params, time)
    % Extracting parameters
    box_length = system_params.box_length;
    box_width = system_params.box_width;
    propeller_radius = system_params.propeller_radius;
    propeller_distance = system_params.propeller_distance;

    % Drone position
    x = position(1);
    y = position(2);
    theta = position(3);
    

    % Define vertices of the box representing the drone
    box_vertices = [x-box_length/2, y-box_width/2;
                    x+box_length/2, y-box_width/2;
                    x+box_length/2, y+box_width/2;
                    x-box_length/2, y+box_width/2;];

    % Define vertices of the circles representing propellers
    front_propeller_center = [x+propeller_distance/2, y, z];
    back_propeller_center = [x-propeller_distance/2, y, z];
    [front_propeller_vertices_x, front_propeller_vertices_y, front_propeller_vertices_z] = cylinder(propeller_radius, 50);
    [back_propeller_vertices_x, back_propeller_vertices_y, back_propeller_vertices_z] = cylinder(propeller_radius, 50);

    % Translate propeller vertices to correct position
    front_propeller_vertices_x = front_propeller_vertices_x + front_propeller_center(1);
    front_propeller_vertices_y = front_propeller_vertices_y + front_propeller_center(2);
    front_propeller_vertices_z = front_propeller_vertices_z * propeller_radius + front_propeller_center(3);
    back_propeller_vertices_x = back_propeller_vertices_x + back_propeller_center(1);
    back_propeller_vertices_y = back_propeller_vertices_y + back_propeller_center(2);
    back_propeller_vertices_z = back_propeller_vertices_z * propeller_radius + back_propeller_center(3);

    % Plotting
    figure;
    hold on;
    % Plot drone box
    plot3(box_vertices(:,1), box_vertices(:,2), box_vertices(:,3), 'b', 'LineWidth', 2);
    % Plot front propeller
    surf(front_propeller_vertices_x, front_propeller_vertices_y, front_propeller_vertices_z, 'FaceColor', 'r', 'EdgeColor', 'none');
    % Plot back propeller
    surf(back_propeller_vertices_x, back_propeller_vertices_y, back_propeller_vertices_z, 'FaceColor', 'r', 'EdgeColor', 'none');
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(['Drone Position at Time = ', num2str(time)]);
    view(3);
    hold off;
end
