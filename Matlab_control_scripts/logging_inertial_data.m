
%> @file
%> @brief Matlab script for logging accelerometer and gyroscope readings to
%> binary file.
%>
%> @details This script will start up the com-port. Command the
%> microcontroller to start outputting the accelerometer readings and which
%> it will save to a the file xxx.
%>
%> 	\authors John-Olof Nilsson, Isaac Skog
%>	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)

%> @defgroup os_matlab_scripts OpenShoe Matlab scripts	
%> @brief Matlab scripts for controlling the OpenShoe system.
%> @details The scripts does not include any functions and therefore is not
%> documented well by the doxygen. See script files for further
%> documentation.
%> @{

% Clear workspace
clear all, clc, close all

% Open serial port
com = serial('COM7', 'BaudRate', 115200,'InputBufferSize',10000);
fopen(com);

% Open binary file for saving inertial data
file = fopen('imu_data.bin', 'w');

% Tell OpenShoe to output inertial data at every interrupt
fwrite(com,[34 1 0 35],'uint8');
fread(com,4,'uint8');

% Open dummy figure with pushbutton such that logging can be aborted
abort = 0;
figure;
uicontrol('style','push','string','Abort data logging','callback','abort=1;');
drawnow

% Logg data until pushbutton pressed
nr_bytes = 0;
while abort==0
    if com.BytesAvailable>0
        nr_bytes = nr_bytes + fwrite(file,fread(com,com.BytesAvailable,'uint8'),'uint8');
    end
    drawnow
end

% Stop output
fwrite(com,[33 0 33],'uint8');

% Close serial port and file
fclose(com);
fclose(file);


% OPEN AND DISPLAY DATA
file = fopen('imu_data.bin','r');

% rx = fread(file,'uint8');
% disp(rx')
% fseek(file,0,'bof');

nr_inertial = floor(nr_bytes/28);

headers_and_plc = zeros(1,ceil(nr_inertial));
inertial_data = zeros(6,ceil(nr_inertial));
checksums = zeros(1,ceil(nr_inertial));
i=1;
while i<=nr_inertial
    headers_and_plc(i) = fread(file,1,'uint16');
    inertial_data(:,i) = fread(file,6,'float',0,'b');
    checksums(i) = fread(file,1,'uint16');
    i=i+1;
end
fclose(file);

figure(1);
plot(inertial_data(1:3,:)');
title('Accelerometer readings');
xlabel('sample number')
ylabel('a [m/s^2]');
figure(2);
plot(inertial_data(4:6,:)'*180/pi);
title('Gyroscope readings');
xlabel('sample number')
ylabel('\omega [deg/s]');

% Save data in ascii format compatible with previous data
file = fopen('data_inert.txt', 'w');
for i=1:32
    fprintf(file,'dummy_header ');
end
fprintf(file,'\r\n');
for i=1:size(inertial_data,2)
    fprintf(file,'%x  %f %f %f  %f %f %f  %i %x  %i  %f %f %f  %f  %f %f %f\r\n',[0 inertial_data(:,i)' 0 0 0 0 0 0 0 0 0 0]);
end

%> @}
