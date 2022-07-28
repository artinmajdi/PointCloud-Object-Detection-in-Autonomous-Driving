# autoencoder

## Description&#x20;

When coupled, lidar and radar offer a potent tool for enhancing the accuracy of radar data. In order to make use of the information in Lidar, we will first remove the ground from the data using the MATLAB lidar toolbox, and then we will use the generated point clouds as the label for our radar data. By doing this, we will force the model to reconstruct the radar data so that it resembles Lidar more.&#x20;

This, however, is just one approach to using lidar data. There are various methods where the lidar and radar data are used in conjunction with each other or where the information is used to enhance the fusing methods. However, we only use the first method since our main goal is to improve the radar data.&#x20;

## Implementation&#x20;

1. Removing the Lidar ground using the MATLAB Lidar toolbox. [reference](https://www.mathworks.com/help/lidar/ref/segmentgroundsmrf.html)&#x20;
2. Initialize a model by training it on public clean radar and LIDAR data.&#x20;
3. Train a model using radar points as input and the new LIDAR points as output.&#x20;
4. Test the trained model on new test samples&#x20;

{% embed url="https://www.mathworks.com/help/lidar/ref/segmentgroundsmrf.html" %}
MATLAB Document
{% endembed %}

{% code title="lidar_ground_removal_loop.m" %}
```matlab
% lidar ground removal

clc; clear all; close all;


% Example 0
% data_dir = 'Y:/RawData/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data/20210622_Sys362_64N_1Tx_Angle_Finding_Raw_Data_LidarCam/Part_0';
% test = '0El0Az_25m_2021-06-22-12-31-08';


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
```
{% endcode %}





