classdef class_time_values
    properties
        subfuncs
    end

    methods

        function obj = class_time_values()
            obj.subfuncs = funcs();

        end

        function obj = draw_one_frame(obj, example, frame_num, mode)
        
            [dir,~] = get_example_directory(example);
        
            [radar_time , lidar_time , camera_time] = obj.subfuncs.load_time_values(dir);
        
            visualize(mode, dir, frame_num, radar_time, lidar_time , camera_time)

        end
   

        function loop_over_all_frames(obj, example_list, do_write, mode)
        
            for example = example_list
        
                [dir,dataset_name] = get_example_directory(example);
                % Dir = uigetdir('U:\ROS\'); % Data Directory
        
                [radar_time , lidar_time , camera_time] = obj.subfuncs.load_time_values(dir);
        
                if do_write == true
                    v = write_settings(mode, dir, dataset_name);
                end
        
        
                num_frames = 200; % size(radar_time,1)
                for i = 1:num_frames
                    clc, disp(['Example:',num2str(example),'  Frame:(',num2str(i),'/',num2str(num_frames),')'])
        
                    try
                        visualize(mode, dir, i, radar_time, lidar_time , camera_time)
        
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



    end
end