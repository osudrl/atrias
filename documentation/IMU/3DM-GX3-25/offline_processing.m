%OFFLINE_PROCESSING.m
% Author: Kevin Galloway (based partly on code by Rohan Divekar
%
% Summary: This script is used for offline processing of the IMU data from
% the Microstrain 3DM-GX3-25. It was originally written for data that had
% been collected using a Simulink model which handled the serial
% communication and packing of the data stream into a matrix, where each
% row of the matrix corresponded to a potential packet from the original
% data stream. That is why some of the code in this file seems a bit
% redundant. (I.e. unpack the matrix to a vector, and then later packe it
% back into a matrix.) IF YOUR CODE FOR HANDLING THE SERIAL COMMUNICATION
% SIMPLY STORES THE DATA AS ONE LONG VECTOR, YOU CAN PROBABLY COMMENT OUT
% SOME OF THE INITIAL CODE IN THIS FILE.

clear all
close all
clc

%Preferences and settings

%Specify the type of data you've collected from the IMU. Currently this
%code is only written to handle the first two data types
data_type = 2; %1->Euler angles; 2->Euler angles and angular rates; 3->Euler angles with mux signal; 4->Orientation matrix; 5->Accel + AngRate + OrientationMatrix
plot_in_degrees = true; %Flag to specify whether to plot in degrees or radians

%% First run an experiment and collect the data stream


%% Initial state of offline processing
if(1)
    experiment_day = '2012-06-12';
    experiment_number = 2
    folder_name = strcat(experiment_day,'/');
    mkdir(experiment_day)
    data_file_name = [folder_name,'Experiment',int2str(experiment_number)];
else
    %Specify the name of the file to load
end

switch data_type
    case 1
        header = 206;
        package_length = 19;
    case 2
        header = 207;
        package_length = 31;
end

if(0)
    state = tg.OutputLog;
    time = tg.TimeLog;
else
    load(data_file_name,'state','time')
end

%If necessary, reshape the input so that it is one long vector of uint8
% This is necessary when working with our xPC Target setup on MABEL,
% because the data output is packaged as a matrix where each row contains a
% full packet (e.g. 19 bytes for the Euler Angles data type). For
% processing here, I wanted it to appear as a single stream of uint8 (as it
% would coming directly out of the IMU)
[rows_state,columns_state] = size(state);
Data_Stream = reshape(state',rows_state*(columns_state),1);
Data_Stream_uint8 = uint8(Data_Stream);

%% Checksum calculations, parse the data stream into packets and stack into a matrix
unmatched_checksum_counter = 0;
unmatched_checksum_indices = [];
j=1;
i=1;
l=length(Data_Stream_uint8);
while (i<l-1)
    if Data_Stream_uint8(i)==header %Look for packet headers
        if (i+package_length)>l %Checks if this is at the end of the data stream
            break;
        end
        possible_packet = Data_Stream_uint8(i:i+(package_length-1));
        
        %Calculate checksum to see if this is a good packet
        possible_packet_uint16 = uint16(possible_packet); %Cast to uint16 for checksum calculation
        check_sum_calculated = sum(possible_packet_uint16(1:package_length-2));
        check_sum_received_big_endian = typecast(possible_packet(end-1:end),'uint16');
        check_sum_received_little_endian = swapbytes(check_sum_received_big_endian);
        %Check if the calculated checksum is different from the received
        %checksum. I got better results with the following syntax than by checking ~= 0.
        if check_sum_calculated - check_sum_received_little_endian > 0 || check_sum_received_little_endian - check_sum_calculated  > 0
            unmatched_checksum_counter = unmatched_checksum_counter+1;
            unmatched_checksum_indices = [unmatched_checksum_indices i];
        end
        %This is now packaged back into matrix form. This is redundant, but
        %the idea was to be able to accept a variety of inputs (i.e. either
        %a vector or a matrix, etc.) Note that I accept every packet, even
        %if the checksum was off. You could modify this to reject packets
        %if the checksum doesn't match.
        Data_Matrix_With_Header(j,1:package_length)=Data_Stream_uint8(i:i+(package_length-1));
        i = i+package_length;
        checksum_diff(j) = max([check_sum_calculated - check_sum_received_little_endian, check_sum_received_little_endian-check_sum_calculated]);
        j=j+1;
    else
        i=i+1;
    end
    %Print an update every once in a while so you don't get too bored
    %waiting.
    if ~rem(i,10000)
        disp([int2str(i) ' out of ' int2str(l)])
    end
end

%% Interpret packets
% %Prep for input to processing model
% Data_Matrix_With_Header_double = double(Data_Matrix_With_Header);
% [data_entries,temp] = size(Data_Matrix_With_Header_double);
% New_Timer = 1:data_entries;
% Final_Input_double = [New_Timer' Data_Matrix_With_Header_double];

save(data_file_name)

%Parse data into useable quantities
% THIS IS THE KEY PART OF THE CODE
% To properly interpret, we first typecast as a 32-bit float (or integer,
% in the case of the time stamp) and then have to swap the bytes, due to
% the Endian difference
load(data_file_name)
[rows_DataMatrix,columns_DataMatrix] = size(Data_Matrix_With_Header);
for i=1:rows_DataMatrix
    IMU_time_stamp_raw(i) = double(swapbytes(typecast(Data_Matrix_With_Header(i,package_length-5:package_length-2),'uint32')));
    RollX(i) = swapbytes(typecast(Data_Matrix_With_Header(i,2:5),'single'));
    PitchY(i) = swapbytes(typecast(Data_Matrix_With_Header(i,6:9),'single'));
    HeadingZ(i) = swapbytes(typecast(Data_Matrix_With_Header(i,10:13),'single'));
    if data_type == 2
        ARX(i) = swapbytes(typecast(Data_Matrix_With_Header(i,14:17),'single'));
        ARY(i) = swapbytes(typecast(Data_Matrix_With_Header(i,18:21),'single'));
        ARZ(i) = swapbytes(typecast(Data_Matrix_With_Header(i,22:25),'single'));
    end
end

%Process IMU Time Stamp and determine the data rate and number of dropped
%packets
IMU_time_stamp_seconds = IMU_time_stamp_raw/62500; %IMU timestamps in seconds
first_good_ind = find(IMU_time_stamp_seconds>0 & IMU_time_stamp_seconds < 1e3,1); %Look for the first valid time-stamp
IMU_time_stamp_good = IMU_time_stamp_seconds(first_good_ind:end);
IMU_time_stamp_absolute = IMU_time_stamp_good-IMU_time_stamp_good(1);
IMU_time_stamp_diff = diff(IMU_time_stamp_absolute);
IMU_time_stamp_diff_nonzero = IMU_time_stamp_diff(IMU_time_stamp_diff~=0 & abs(IMU_time_stamp_diff)<1);
% figure; hist(IMU_time_stamp_diff_nonzero,60)
% title(['Histogram of time-stamp separation for Experiment ', int2str(experiment_number), ' on ', experiment_day])
imu_sampling_period = mode(round(1000*IMU_time_stamp_diff_nonzero)/1000);
imu_data_rate = 1/imu_sampling_period;
disp(['IMU data rate is ',num2str(imu_data_rate),'Hz'])
dropped_packet_ind = find(IMU_time_stamp_diff > imu_sampling_period + .001);
good_packet_ind = find(IMU_time_stamp_diff > imu_sampling_period - .001 & IMU_time_stamp_diff < imu_sampling_period + .001);
num_good_packets = length(good_packet_ind);
num_dropped_packets = length(dropped_packet_ind);
percent_dropped = (num_dropped_packets/(num_good_packets + num_dropped_packets))*100;
disp(['Dropped ',num2str(percent_dropped),'% of IMU packets']);

%% Plot results
if plot_in_degrees
    scale_factor = 180/pi;
else
    scale_factor = 1;
end

figure('Name','RollX','NumberTitle','off'); plot(IMU_time_stamp_absolute,RollX(first_good_ind:end)*scale_factor); title('IMU RollX');xlabel('Time stamp (sec)');
if plot_in_degrees
    ylabel('Degrees')
else
    ylabel('Radians')
end

figure('Name','PitchY','NumberTitle','off'); plot(IMU_time_stamp_absolute,PitchY(first_good_ind:end)*scale_factor); title('IMU PitchY');xlabel('Time stamp (sec)');
if plot_in_degrees
    ylabel('Degrees')
else
    ylabel('Radians')
end

figure('Name','HeadingZ','NumberTitle','off'); plot(IMU_time_stamp_absolute,HeadingZ(first_good_ind:end)*scale_factor); title('IMU HeadingZ');xlabel('Time stamp (sec)');
if plot_in_degrees
    ylabel('Degrees')
else
    ylabel('Radians')
end

if data_type == 2
    figure('Name','ARX','NumberTitle','off'); plot(IMU_time_stamp_absolute,ARX(first_good_ind:end)*scale_factor); title('IMU ARX');xlabel('Time stamp (sec)');
    if plot_in_degrees
        ylabel('Degrees/sec')
    else
        ylabel('Radians/sec')
    end
    
    figure('Name','ARY','NumberTitle','off'); plot(IMU_time_stamp_absolute,ARY(first_good_ind:end)*scale_factor); title('IMU ARY');xlabel('Time stamp (sec)');
    if plot_in_degrees
        ylabel('Degrees/sec')
    else
        ylabel('Radians/sec')
    end
    
    figure('Name','ARZ','NumberTitle','off'); plot(IMU_time_stamp_absolute,ARZ(first_good_ind:end)*scale_factor); title('IMU ARZ');xlabel('Time stamp (sec)');
    if plot_in_degrees
        ylabel('Degrees/sec')
    else
        ylabel('Radians/sec')
    end
end