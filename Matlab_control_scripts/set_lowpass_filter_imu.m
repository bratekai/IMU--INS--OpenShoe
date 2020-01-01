
%> @file
%> @brief Matlab script for changing number of filter taps of internal low
%> pass filter in IMU.
%>
%> @details This script starts up the com-port and send a command to the
%> OpenShoe system to change to number of filter taps of the internal low
%> pass filter in the IMU. The number of filter taps is an argument of the
%> sent command.
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

% Log2 of the number of desired filter taps
% i.e. 0->1, 1->2 2->4 3->8 4->16
% 4 (16) is the maximum value.
log2_filter_taps = 0;

% Set number of filter taps of IMU internal low pass filter
fwrite(com,[19 log2_filter_taps 0 19+log2_filter_taps],'uint8');
% Read ack
fread(com,4,'uint8');
% Close com-port
fclose(com);

%> @}