% lidar ground removal

clc; clear; close all;

obj = funcs();
num_frames = 200;

for example = 1:4
    
    folder_dir = get_example_directory(example);    
    mkdir([folder_dir, '/Lidar_r/'])
    
    for i = 1:num_frames
        clc
        disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
    
        file_dir_in  = [folder_dir, '/Lidar/', num2str(i), '_.txt'];
        file_dir_out = [folder_dir, '/Lidar_r/', num2str(i), '_.txt'];
        lidar_r = obj.removing_lidar_ground(file_dir_in, file_dir_out);
    end

end

