% lidar ground removal

clc; clear all; close all;


% Example 0
% data_dir = 'Y:/RawData/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data_LidarCam/Part_0';
% test = '0El0Az_25m_2021-06-22-12-31-08';


num_frames = 100;

for example = 1:5
    
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