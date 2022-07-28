# Lidar Ground Removal

# ****Description**

When coupled, lidar and radar offer a potent tool for enhancing the accuracy of radar data. In order to make use of the information in Lidar, we will first remove the ground from the data using the MATLAB lidar toolbox, and then we will use the generated point clouds as the label for our radar data. By doing this, we will force the model to reconstruct the radar data so that it resembles Lidar more.

This, however, is just one approach to using lidar data. There are various methods where the lidar and radar data are used in conjunction with each other or where the information is used to enhance the fusing methods. However, we only use the first method since our main goal is to improve the radar data.

# ****Implementation**

1. Removing the Lidar ground using the MATLAB Lidar toolbox. [reference](https://www.mathworks.com/help/lidar/ref/segmentgroundsmrf.html) 
    
    [segmentGroundSMRF](https://www.mathworks.com/help/lidar/ref/segmentgroundsmrf.html)
    
2. Initialize a model by training it on public clean radar and LIDAR data.
3. Train a model using radar points as input and the new LIDAR points as output.
4. Test the trained model on new test samples.

# Implementations

[https://github.com/artinmajdi/autoencoder.git](https://github.com/artinmajdi/autoencoder.git)

```matlab
clc; clear all; close all;

num_frames = 200;

for example = 1:4
    
    folder_dir = get_example_directory(example);    
    mkdir([folder_dir, '/Lidar_r/'])
    
    for i = 1:num_frames
        clc
        disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
    
        file_dir_in  = [folder_dir, '/Lidar/', num2str(i), '_.txt'];
        file_dir_out = [folder_dir, '/Lidar_r/', num2str(i), '_.txt'];
        lidar_r = removing_lidar_ground(file_dir_in, file_dir_out);
    end

end
```

```matlab
function lidar_r = removing_lidar_ground(file_dir_in, file_dir_out)

    lidar = load(file_dir_in);
    x = lidar(:, 1);
    y = lidar(:, 2);
    z = lidar(:, 3);
    intensity = lidar(:, 4);
    tm = lidar(:, 5);
    ix = lidar(:, 6);
    [x, y] = lidar_rotation(x, y);
    
    % camera = imread([folder_dir, '/Camera/', num2str(i), '_.jpg']);
    
    pc = pointCloud([y, x, z]);
    groundPtsIdx = segmentGroundSMRF(pc);
    pc_ground_removed = select(pc, ~groundPtsIdx);
    xr = pc_ground_removed.Location(:, 1);
    yr = pc_ground_removed.Location(:, 2);
    zr = pc_ground_removed.Location(:, 3);
    intensity_r = intensity(~groundPtsIdx);
    tm_r = tm(~groundPtsIdx);
    ix_r = ix(~groundPtsIdx);
    
    
    % figure, subplot(121) %, imshow(camera)
    % scatter3(xr, yr, zr, 1, 'filled', 'w');
    % view([0, 90]), colorbar, caxis([0, 40])
    

    % save([folder_dir , '/' , num2str(i) , '_coordinates.mat'],'x','y','z','xr','yr','zr')
    
    lidar_r = [xr,yr,zr,intensity_r,tm_r,ix_r];
    writematrix(lidar_r , file_dir_out)

end
```

```matlab
close all; clear all; clc; colmap = parula;

write = false;

for example = 1%:4

    [Dir,dataset_name] = get_example_directory(example);
    
    % Dir = uigetdir('U:\ROS\'); % Data Directory
    
    radar_time = load([Dir '/Radar_time_tag.txt']); % Radar Time File
    lidar_time = load([Dir '/Lidar_time_tag.txt']); % Lidar Time File
    camera_time = load([Dir '/Camera_2_time_tag.txt']); %Camera Time File
    
    if write == true
        v=VideoWriter([Dir,'/',dataset_name,'_lidar_only.avi']);
        v.FrameRate=2;
        v.Quality=50;
        open(v);
    end

    num_frames = 200; % size(radar_time,1)

    for i = 1%:num_frames 
        
        % clc
        % disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
        
        try
            visualize(Dir, i, radar_time, lidar_time , camera_time)
%             visualize_lidar_only(Dir, i, radar_time, lidar_time , camera_time)
            
            drawnow();
            
            if write == true
                frame=getframe(gcf);
                writeVideo(v,frame); 
            end

        catch
            continue
        end
        
    end
%     close all;
%     close(v)

end

function visualize(Dir, i, radar_time, lidar_time , camera_time)
    
    colmap = parula;

    % Load Synchronized Data
    radar = load([Dir '\Radar\' num2str(radar_time(i,1)) '_.txt']); % Load Radar Frame
    [diff,ind] = min(abs(lidar_time(:,2)-radar_time(i,2))); % Find Synchronized Lidar Frame
    
    lidar_r = load([Dir '\Lidar_r\' num2str(ind) '_.txt']); % Load Lidar Frame
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

    scatter3(lidar_r(:,2),lidar_r(:,1),lidar_r(:,3),3,'w','filled') % Plot Lidar Data
    hold off

    view(2) % Set View to BirdsEye
    c = colorbar;
    xlim([-150 150]); ylim([0 300]); zlim([-50,50]); caxis([30 80]); % Plot Axis Limits
    xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');c.Label.String = 'RCS(dB)';

    % Camera Image
    subplot(2,4,[4,8])
    imshow(camera); % Plot Camera Image
    sgtitle([Dir '    Frame ' num2str(i-1)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end

function visualize_lidar_only(Dir, i, radar_time, lidar_time , camera_time)
    
    colmap = parula;

    % Load Synchronized Data
    % radar = load([Dir '\Radar\' num2str(radar_time(i,1)) '_.txt']); % Load Radar Frame
    [diff,ind] = min(abs(lidar_time(:,2)-radar_time(i,2))); % Find Synchronized Lidar Frame
    
    lidar = load([Dir '\Lidar\' num2str(ind) '_.txt']); % Load Lidar Frame
    [diff,ind] = min(abs(camera_time(:,2)-radar_time(i,2))); % Find Synchronized Camera Image

    lidar_r = load([Dir '\Lidar_r\' num2str(ind) '_.txt']); % Load Lidar Frame

    camera = imread([Dir '\Camera\' num2str(ind) '_.jpg']); % Load Camera Image
    

    % Plot Syncronized Data
    figure(1)
    set(gcf, 'Position', get(0, 'Screensize')); 
    set(gcf, 'InvertHardcopy', 'off')

    subplot(131)

    scatter3(-lidar(:,2),lidar(:,1),lidar(:,3),3,'w','filled') % Plot Lidar Data
    set(gca,'Color',[colmap(1,:)]);grid on;
    set(gca,'linewidth',2,'fontsize',15,'fontweight','bold');

    view(2) % Set View to BirdsEye
    c = colorbar;
    xlim([-150 150]); ylim([0 300]); zlim([-50,50]); caxis([30 80]); % Plot Axis Limits
    xlabel('X(m)');ylabel('Y(m)');zlabel('Z(m)');

    % Lidar with removed background Image
    subplot(132)
    scatter3(lidar_r(:,2),lidar_r(:,1),lidar_r(:,3),3,'w','filled') % Plot Lidar Data
    set(gca,'Color',[colmap(1,:)]);grid on;
    set(gca,'linewidth',2,'fontsize',15,'fontweight','bold');

    view(2) % Set View to BirdsEye
    c = colorbar;
    xlim([-150 150]); ylim([0 300]); zlim([-50,50]); caxis([30 80]); % Plot Axis Limits
    xlabel('X(m)');c.Label.String = 'RCS(dB)';
%     axis off

    subplot(133)
    imshow(camera); % Plot Camera Image

    sgtitle([Dir '    Frame ' num2str(i-1)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end
```