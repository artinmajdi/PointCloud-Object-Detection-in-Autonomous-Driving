function data_dir = get_example_directory(example)
    switch example
        case 1
            data_dir = 'U:/ROS\20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            test     = 'Downtown_University_2022-03-29-11-04-56_3';
    
        case 2
            
            data_dir = 'U:/ROS/20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            test     = 'Freeway_2022-03-29-11-17-15_1';
    
        case 3
            data_dir = 'U:/ROS/20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            test     = 'Urban_Costco_2022-03-29-11-32-55_2';
            
        case 4
            data_dir = 'U:/ROS\20220329_Sys362_Nvidia_RoadData/256F2TX_RoadData';
            test     = 'Downtown_University_2022-03-29-11-04-56_3';
            
        case 5
            data_dir = 'U:/ROS/20220126_Sys363_LWSRQT2/256F2Tx\Dynamic/Away';
            test = 'Away_3_2022-01-26-09-57-10';

    end

    data_dir = [data_dir, '/' , test];
end