function [x_, y_] = lidar_rotation(x, y)
    
    % rotate CW 90
    angle = 90;
    % R = [cosd(angle), -sind(angle); ...
    %     sind(angle),  cosd(angle)];
    % xy_R = R*[x'; y'];
    % x_ = xy_R(1, :);
    % y_ = xy_R(2, :);
    
    x_ = x*cosd(angle) - y*sind(angle);
    y_ = x*sind(angle) + y*cosd(angle);

end
