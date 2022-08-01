% lidar ground removal

clc; clear; close all;

obj = funcs();
frame_list = 1:200;

for example = 1:4
    
    folder_dir = get_example_directory(example);    
    mkdir([folder_dir, '/Lidar_r/'])
    
    for i = frame_list
        clc
        disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(length(frame_list)),')'])
    
        file_dir_in  = [folder_dir, '/Lidar/', num2str(i), '_.txt'];
        file_dir_out = [folder_dir, '/Lidar_r/', num2str(i), '_.txt'];
        lidar_r = obj.removing_lidar_ground(file_dir_in, file_dir_out);
    end

end

