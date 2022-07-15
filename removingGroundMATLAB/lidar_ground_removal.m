% lidar ground removal

clc; clear all; close all;


data_dir = '/mnt/u/RawData/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data_LidarCam/Part_0';
test = '0El0Az_25m_2021-06-22-12-31-08';

folder_dir = [data_dir, '/', test];

for i = 1
    lidar = load([folder_dir, '/Lidar/', num2str(i), '_.txt']);
    x = lidar(:, 1);
    y = lidar(:, 2);
    z = lidar(:, 3);
    intensity = lidar(:, 4);
    [x, y] = lidar_rotation(x, y);
    
%     camera = imread([folder_dir, '/Camera/concat_', num2str(i), '.jpg']);
    
    pc = pointCloud([x, y, z]);
    groundPtsIdx = segmentGroundSMRF(pc);
    pc_ground_removed = select(pc, ~groundPtsIdx);
    xr = pc_ground_removed.Location(:, 1);
    yr = pc_ground_removed.Location(:, 2);
    zr = pc_ground_removed.Location(:, 3);
    intensity_r = intensity(~groundPtsIdx);
    
    
%     figure, 
%     scatter3(xr, yr, zr, 1, 'filled', 'w');
%     view([0, 90]), colorbar, caxis([0, 40])

    save([string(i) , '_coordinates.mat'],'x','y','z','xr','yr','zr')
    
end
