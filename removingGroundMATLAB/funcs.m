function obj = funcs

    disp('funcs is loaded')
    obj.draw_one_frame = @draw_one_frame;
    obj.loop_over_all_frames = @loop_over_all_frames;
    obj.load_time_values = @load_time_values;
    obj.visualize = @visualize;
    obj.removing_lidar_ground = @removing_lidar_ground;

end

function draw_one_frame(example, frame_num, mode)

    [dir,~] = get_example_directory(example);

    times = load_time_values(dir);

    visualize(mode, dir, frame_num, times)
end

function loop_over_all_frames(example_list, frame_list, do_write, mode)

    for example = example_list

        [dir,dataset_name] = get_example_directory(example);
        % dir = uigetdir('U:\ROS\'); % Data directory

        times = load_time_values(dir);

        if do_write == true
            v = write_settings(mode, dir, dataset_name);
        end


        % frame_list = 1:size(times.radar,1)
        for i = frame_list
            clc, disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(length(frame_list)),')'])

            try
                visualize(mode, dir, i, times)

                if do_write == true
                    frame=getframe(gcf);
                    writeVideo(v,frame);
                end
            catch
                continue
            end

        end

        close all;

        if do_write == true
            close(v)
        end

    end

end

function times = load_time_values(dir)

    times.radar = load([dir '/radar_time_tag.txt']); % Radar Time File
    times.lidar = load([dir '/lidar_time_tag.txt']); % Lidar Time File
    times.camera = load([dir '/Camera_2_time_tag.txt']); %Camera Time File

end

function v = write_settings(mode, dir, dataset_name)

    switch mode
        case 'lidar_w_wo_ground'
            post_tag = '_lidar_only';

        case 'radar_lidar_fused'
            post_tag = '';
    end

    v=VideoWriter( [dir, '/', dataset_name, post_tag, '.avi'] );
    v.FrameRate=2;
    v.Quality=100;
    open(v);

end

function visualize(mode, dir, frame_index, times)

    switch mode
        case 'lidar_w_wo_ground'
            visualize_lidar_w_wo_ground(dir, frame_index, times)

        case 'radar_lidar_fused'
            visualize_radar_lidar_fused(dir, frame_index, times)
    end

    drawnow();

end

function visualize_radar_lidar_fused(dir, i, times)

    % ---------------------------Load RADAR times------------------------------------------ %
    radar = load([dir '\Radar\' num2str(times.radar(i,1)) '_.txt']); % Load Radar Frame


    % ---------------------------Load Synchronized LIDAR w/wo background times ------------ %
    [~,ind_lidar] = min(abs(times.lidar(:,2)-times.radar(i,2))); % Find Synchronized Lidar Frame
    lidar_r = load([dir '\Lidar_r\' num2str(ind_lidar) '_.txt']); % Load Lidar Frame


    % ---------------------------Load Synchronized CAMERA times --------------------------- %
    [~,ind_camera] = min(abs(times.camera(:,2)-times.radar(i,2))); % Find Synchronized Camera Image
    camera = imread([dir '\Camera\' num2str(ind_camera) '_.jpg']); % Load Camera Image



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
        sgtitle([dir '    Frame ' num2str(i) '/' num2str(ind_lidar)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end

function visualize_lidar_w_wo_ground(dir, i, times)

    % Load Synchronized LIDAR w/wo background --------------------------------------- %
    [diff,ind] = min(abs(times.lidar(:,2)-times.radar(i,2)));
    lidar   = load([dir '\Lidar\'   num2str(ind) '_.txt']);
    lidar_r = load([dir '\Lidar_r\' num2str(ind) '_.txt']);


    % Load Synchronized CAMERA ------------------------------------------------------ %
    [diff,ind] = min(abs(times.camera(:,2)-times.radar(i,2))); % Find Synchronized Camera Image
    camera = imread([dir '\Camera\' num2str(ind) '_.jpg']); % Load Camera Image


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
        sgtitle([dir '    Frame ' num2str(i) '/' num2str(ind)],'fontsize',15,'fontweight','bold','Interpreter', 'none')

end

function scatter3_plot(pointcloud, bird_view, mode)
    switch mode
        case 'radar'
            scatter3(-pointcloud(:,4),pointcloud(:,3),pointcloud(:,5),10,pointcloud(:,9),'filled')

        case 'lidar'
            scatter3(-pointcloud(:,2),pointcloud(:,1),pointcloud(:,3),3,'white','filled')
            plotting_settings(bird_view)
    end
end

function plotting_settings(bird_view)

    colmap = parula;

    set(gca,'Color',[colmap(1,:)]);
    grid on;
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

    lidar_r = [xr,-yr,zr,intensity_r,tm_r,ix_r];
    writematrix(lidar_r , file_dir_out)

end

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
