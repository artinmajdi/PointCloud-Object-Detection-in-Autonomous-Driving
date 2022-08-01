% lidar ground removal

clc; clear; close all;


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