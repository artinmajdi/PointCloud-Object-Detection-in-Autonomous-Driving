% lidar ground removal

clc; clear all; close all;


% Example 0
% data_dir = 'Y:/RawData/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data_LidarCam/Part_0';
% test = '0El0Az_25m_2021-06-22-12-31-08';


num_frames = 20;

for example = 1:5
    
    folder_dir = get_example_directory(1);
    % folder_dir = [data_dir, '/', test];
    
    mkdir([folder_dir, '/Lidar_r/'])
    
    for i = 1:num_frames
        clc
        disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
    
        lidar = load([folder_dir, '/Lidar/', num2str(i), '_.txt']);
        x = lidar(:, 1);
        y = lidar(:, 2);
        z = lidar(:, 3);
        intensity = lidar(:, 4);
        tm = lidar(:, 5);
        ix = lidar(:, 6);
        [x, y] = lidar_rotation(x, y);
        
        % camera = imread([folder_dir, '/Camera/', num2str(i), '_.jpg']);
        
        pc = pointCloud([x, y, z]);
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
        
        % xr_sz = size(xr);
        % lidar_r = zeros([xr_sz(1),6]);
        % lidar_r(:,1) = xr;
        % lidar_r(:,2) = yr;
        % lidar_r(:,3) = zr;
        % lidar_r(:,4) = intensity_r;
        % lidar_r(:,5) = tm_r;
        % lidar_r(:,6) = ix_r;
        lidar_r = [xr,yr,zr,intensity_r,tm_r,ix_r];
        writematrix(lidar_r , [folder_dir, '/Lidar_r/', num2str(i), '_.txt'])
        
    end

end