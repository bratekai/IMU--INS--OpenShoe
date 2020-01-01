
%> @file
%> @brief Matlab script for executing OpenShoe calibration routines.
%>
%> @details This script will start up the com-port, send a command to the
%> OpenShoe system to initialize its gyro/accelerometer calibration
%> routine. For the gyro calibration routine, once this command has been
%> sent the system should be left stationarry for at least 15[s]. For the
%> acceleromter calibration routine, the user will be asked to place the
%> system is different orientations.
%>
%> 	\authors John-Olof Nilsson, Isaac Skog
%>	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)

%> @defgroup os_matlab_scripts OpenShoe Matlab scripts	
%> @brief Matlab scripts for controlling the OpenShoe system.
%> @{

clear all, clc

% Open serial port
com = serial('COM7', 'BaudRate', 115200,'InputBufferSize',10000);
fopen(com);

% Gyro calibration
fwrite(com,[17 0 17],'uint8');
% Read ack
fread(com,4,'uint8');
disp('Leave IMU stationary for at least 15s');

% Number of orientations used for the accelerometer calibration
% Minimum 3
% nr_orientations = 3;
% Accelerometer calibration
%fwrite(com,[18 nr_orientations 0 18+nr_orientations],'uint8');
% Read ack
% fread(com,4,'uint8');
% for i=1:nr_orientations
%     while true
%         if(com.BytesAvailable>0)
%             fread(com,1,'uint8')
%             disp('Put system in new orientation');
%             break;
%         end
%     end
% end

fclose(com);

%> @}