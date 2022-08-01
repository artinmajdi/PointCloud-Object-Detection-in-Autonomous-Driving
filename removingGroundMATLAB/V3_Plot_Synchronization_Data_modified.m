% [Dir,dataset_name] = get_example_directory(2);
% cd(Dir)

close all; clear; clc;

obj = funcs();

mode = 'radar_lidar_fused'; % radar_lidar_fused 'lidar_w_wo_ground'

what_to_do = 'loop_over_all_frames';
switch what_to_do

    case 'draw_one_frame'
        example = 1;
        frame_num = 115;
        obj.draw_one_frame(example, frame_num, mode)

    case 'loop_over_all_frames'
        do_write = false;
        frame_list = 1:20;
        example_list = 1:4;
        obj.loop_over_all_frames(example_list, frame_list, do_write, mode)

end