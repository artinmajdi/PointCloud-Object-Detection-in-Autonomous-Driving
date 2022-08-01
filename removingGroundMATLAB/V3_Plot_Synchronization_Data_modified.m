% [Dir,dataset_name] = get_example_directory(2);
% cd(Dir)

close all; clear; clc; colmap = parula;

write = false;
mode = 'radar_lidar_fused'; % radar_lidar_fused 'lidar_w_wo_ground'


for example = 2:4

    [Dir,dataset_name] = get_example_directory(example);
    % Dir = uigetdir('U:\ROS\'); % Data Directory
    

    [radar_time , lidar_time , camera_time] = load_time_values(Dir);
    
    if write == true
        v = write_settings(mode, Dir, dataset_name);
    end


    num_frames = 200; % size(radar_time,1)
    for i = 1:num_frames         
%         clc, 
        disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
        
        try         
            visualize(mode, Dir, i, radar_time, lidar_time , camera_time)
            
            if write == true
                frame=getframe(gcf);
                writeVideo(v,frame); 
            end
        catch
            continue
        end
        
    end

    close all;

    if write == true
        close(v)
    end

end


function [radar_time , lidar_time , camera_time] = load_time_values(Dir)

    radar_time = load([Dir '/Radar_time_tag.txt']); % Radar Time File
    lidar_time = load([Dir '/Lidar_time_tag.txt']); % Lidar Time File
    camera_time = load([Dir '/Camera_2_time_tag.txt']); %Camera Time File

end

function v = write_settings(mode, Dir, dataset_name)

    switch mode
        case 'lidar_w_wo_ground'
            post_tag = '_lidar_only';
        
        case 'radar_lidar_fused'
            post_tag = '';
    end

    v=VideoWriter( [Dir, '/', dataset_name, post_tag, '.avi'] );
    v.FrameRate=2;
    v.Quality=50;
    open(v);

end

function visualize(mode, Dir, i, radar_time, lidar_time , camera_time)

    switch mode
        case 'lidar_w_wo_ground'
            visualize_lidar_w_wo_ground(Dir, i, radar_time, lidar_time , camera_time)
        
        case 'radar_lidar_fused'
            visualize_radar_lidar_fused(Dir, i, radar_time, lidar_time , camera_time)
    end

    drawnow();

end

function visualize_radar_lidar_fused(Dir, i, radar_time, lidar_time , camera_time)
    
    % ---------------------------Load RADAR times------------------------------------------ %
    radar = load([Dir '\Radar\' num2str(radar_time(i,1)) '_.txt']); % Load Radar Frame


    % ---------------------------Load Synchronized LIDAR w/wo background times ------------ %
    [diff,ind] = min(abs(lidar_time(:,2)-radar_time(i,2))); % Find Synchronized Lidar Frame
    lidar_r = load([Dir '\Lidar_r\' num2str(ind) '_.txt']); % Load Lidar Frame
   

    % ---------------------------Load Synchronized CAMERA times --------------------------- %
    [diff,ind] = min(abs(camera_time(:,2)-radar_time(i,2))); % Find Synchronized Camera Image
    camera = imread([Dir '\Camera\' num2str(ind) '_.jpg']); % Load Camera Image
    


    % ------------------------ Plot Syncronized Data  ------------------------------------- %    
    figure(1)
    set(gcf, 'Position', get(0, 'Screensize')); 
    set(gcf, 'InvertHardcopy', 'off')


        % --------------------Plotting the RADAR & LIDAR points---------------------------- %
        subplot(2,4,[1,2,3,5,6,7])
        scatter3_plot(radar,   false, 'radar'),  hold on
        scatter3_plot(lidar_r, true,  'lidar'),  hold off
        
        
        % --------------------Plotting the CAMERA image------------------------------------ %
        subplot(2,4,[4,8])
        imshow(camera);
        
        % --------------------------------------------------------------------------------- %     
        sgtitle([Dir '    Frame ' num2str(i-1)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end

function visualize_lidar_w_wo_ground(Dir, i, radar_time, lidar_time , camera_time)
    
    % Load Synchronized LIDAR w/wo background --------------------------------------- %
    [diff,ind] = min(abs(lidar_time(:,2)-radar_time(i,2))); 
    lidar   = load([Dir '\Lidar\'   num2str(ind) '_.txt']); 
    lidar_r = load([Dir '\Lidar_r\' num2str(ind) '_.txt']); 


    % Load Synchronized CAMERA ------------------------------------------------------ %
    [diff,ind] = min(abs(camera_time(:,2)-radar_time(i,2))); % Find Synchronized Camera Image
    camera = imread([Dir '\Camera\' num2str(ind) '_.jpg']); % Load Camera Image
    

    % ------------------------Plot Syncronized Data ----------------------------------- %    
    figure(1)
    set(gcf, 'Position', get(0, 'Screensize')); 
    set(gcf, 'InvertHardcopy', 'off')


        % --------------------Plotting the LIDAR points---------------------------------- %
        subplot(131)
        scatter3_plot(lidar, true, 'lidar')

        % --------------------Plotting the LIDAR WITHOUT BACKGROUND points--------------- %
        subplot(132)
        scatter3_plot(lidar_r, true, 'lidar')
        
        % --------------------Plotting the CAMERA image---------------------------------- %
        subplot(133)
        imshow(camera);
    
        % ------------------------------------------------------------------------------- %    
        sgtitle([Dir '    Frame ' num2str(i-1)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end

function scatter3_plot(pointcloud, bird_view, mode)
    switch mode
        case 'radar'
            scatter3(-pointcloud(:,4),pointcloud(:,3),pointcloud(:,5),20,pointcloud(:,9),'filled')

        case 'lidar'
            scatter3(-pointcloud(:,2),pointcloud(:,1),pointcloud(:,3),3,'w','filled') % Plot Lidar Data        
            plotting_settings(bird_view)            
    end
end

function plotting_settings(bird_view)

    colmap = parula;

    set(gca,'Color',[colmap(1,:)]);grid on;
    set(gca,'linewidth',2,'fontsize',15,'fontweight','bold');

    % Set View to BirdsEye
    if bird_view
        view(2) 
    end

    xlim([-100 100]); ylim([0 200]); zlim([-50,50]); caxis([30 80]); % Plot Axis Limits
    xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');

    c = colorbar;
    c.Label.String = 'RCS(dB)';

end

