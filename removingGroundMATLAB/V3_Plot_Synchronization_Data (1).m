close all; clear all; clc; colmap = parula;

Dir = uigetdir('U:\ROS\'); % Data Directory

radar_time = load([Dir '\Radar_time_tag.txt']); % Radar Time File
lidar_time = load([Dir '\Lidar_time_tag.txt']); % Lidar Time File
camera_time = load([Dir '\Camera_2_time_tag.txt']); %Camera Time File

for i = 1:size(radar_time,1)
    
    % Load Synchronized Data
    radar = load([Dir '\Radar\' num2str(radar_time(i,1)) '_.txt']); % Load Radar Frame
    [diff,ind] = min(abs(lidar_time(:,2)-radar_time(i,2))); % Find Synchronized Lidar Frame
    lidar = load([Dir '\Lidar\' num2str(ind) '_.txt']); % Load Lidar Frame
    [diff,ind] = min(abs(camera_time(:,2)-radar_time(i,2))); % Find Synchronized Camera Image
    camera = imread([Dir '\Camera\' num2str(ind) '_.jpg']); % Load Camera Image
    
    % Plot Syncronized Data
    figure(1)
    set(gcf, 'Position', get(0, 'Screensize')); 
    set(gcf, 'InvertHardcopy', 'off')
    % Birds Eye View Plot
    subplot(2,4,[1,2,3,5,6,7])
    scatter3(-radar(:,4),radar(:,3),radar(:,5),20,radar(:,9),'filled') % Plot Radar Data
    set(gca,'Color',[colmap(1,:)]);grid on;
    set(gca,'linewidth',2,'fontsize',15,'fontweight','bold');
    hold on
    scatter3(-lidar(:,2),lidar(:,1),lidar(:,3),3,'w','filled') % Plot Lidar Data
    hold off
    view(2) % Set View to BirdsEye
    c = colorbar;
    xlim([-150 150]); ylim([0 300]); zlim([-50,50]); caxis([30 80]); % Plot Axis Limits
    xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');c.Label.String = 'RCS(dB)';
    % Camera Image
    subplot(2,4,[4,8])
    imshow(camera); % Plot Camera Image
    sgtitle([Dir ', Frame ' num2str(i-1)],'fontsize',15,'fontweight','bold')
    
    drawnow();
end