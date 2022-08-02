function [dir, dataset_name] = get_example_directory(example)

    switch example
        case 1
            % a. Busy downtown road data, slower speed with traffic
            % b. 1425 frames (~2 minutes of data)
            data_dir = 'U:/ROS/20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            dataset_name     = 'Downtown_University_2022-03-29-11-04-56_3';
    
        case 2
            % a. Freeway data at higher speed with barriers on either side of vehicle and overhead structures
            % b. 1424 frames (~2 minutes of data)
            data_dir = 'U:/ROS/20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            dataset_name     = 'Freeway_2022-03-29-11-17-15_1';
    
        case 3
            % a. Costco parking lot with pedestrian targets, parked cars, and static moments
            % b. 1424 frames (~2 minutes of data)      
            data_dir = 'U:/ROS/20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            dataset_name     = 'Urban_Costco_2022-03-29-11-32-55_2';

        case 4
            % a. Static case with dynamic vehicle on boresight. Traffic cones lining the path which the vehicle takes.
            % b. 385 frames (~30 seconds of data)           
            data_dir = 'U:/ROS/20220126_Sys363_LWSRQT2/256F2Tx/Dynamic/Away';
            dataset_name = 'Away_3_2022-01-26-09-57-10';

    end
    
    dir = [data_dir, '/' , dataset_name];

end