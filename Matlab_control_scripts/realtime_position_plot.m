
%> @file
%> @brief Matlab script for real-time plotting of OpenShoe position
%> estimates.
%>
%> @details This script will start up the com-port. Command the
%> microcontroller to initialize the INS and start outputting position
%> estimates. The process is stopped by pressing the push-botton in the
%> plot figure.
%>
%> 	\authors John-Olof Nilsson, Isaac Skog
%>	\copyright Copyright (c) 2011 OpenShoe, ISC License (open source)

%> @defgroup os_matlab_scripts OpenShoe Matlab scripts	
%> @brief Matlab scripts for controlling the OpenShoe system.
%> @{

% Clear workspace
clear all, clc, close all

% Open serial port
com = serial('COM7', 'BaudRate', 115200,'InputBufferSize',10000,'ByteOrder','bigEndian');
fopen(com);
pause(1);

% Figure for plotting
figure(1)
hold on
%grid on
title('Real-time position plot');
xlabel('x [m]');
ylabel('y [m]');
% Add pushbutton such that logging can be aborted
abort_flag = 0;
uicontrol('style','push','string','Stop','callback','abort_flag=1;');
drawnow
pause(0.1)

% Reset INS
fwrite(com,[16 0 16],'uint8');
fread(com,4,'uint8'); % Read ack

% Navigational states (8.192 [Hz])
fwrite(com,[36 9 0 45],'uint8');
fread(com,4,'uint8'); % Read ack


alt_flag = 0;
nav_state = zeros(2,10);
counter = 0;
while abort_flag==0
    if(com.BytesAvailable>47)
        fread(com,1,'uint8'); % Header
        fread(com,1,'uint8'); % Payload size
        nav_state(alt_flag+1,:) = fread(com,10,'float');
        fread(com,1,'uint32'); % Timer
        fread(com,1,'uint16'); % Checksum
        figure(1)
        plot(nav_state(:,1),nav_state(:,2),'b');
%        plot([counter counter+1],-nav_state(:,3),'b');
        hold on;
        axis equal;
        drawnow;
        alt_flag = 1 - alt_flag;
        counter = counter + 1;
    end
end

% Stop output
fwrite(com,[33 0 33],'uint8');

fclose(com);

%> @}
