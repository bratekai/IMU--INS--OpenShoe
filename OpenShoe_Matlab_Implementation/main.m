%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%>
%> @file Main.m
%>                                                                           
%> @brief This is the skeleton file for processing data from a foot mounted
%> IMU. The data is processed using a Kalman filter based zero-velocity 
%> aided inertial navigation system algorithm.
%> 
%> @details This is the skeleton file for processing data data from a foot
%> mounted inertial measurement unit (IMU). The data is processed using a 
%> Kalman filter based zero-velocity aided inertial navigation system. The
%> processing is done in the following order. 
%> 
%> \li The IMU data and the settings controlling the Kalman filter is loaded.
%> \li The zero-velocity detector process all the IMU data.
%> \li The filter algorithm is processed.
%> \li The in data and results of the processing is plotted.
%>
%> @authors Isaac Skog, John-Olof Nilsson
%> @copyright Copyright (c) 2011 OpenShoe, ISC License (open source)
%>
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Loads the algorithm settings and the IMU data
disp('Loads the algorithm settings and the IMU data')
u=settings();


%% Run the zero-velocity detector 
disp('Runs the zero velocity detector')
[zupt T]=zero_velocity_detector(u);


%% Run the Kalman filter
disp('Runs the filter')
[x_h cov]=ZUPTaidedINS(u,zupt);

%% Print the horizontal error and spherical error at the end of the
%% trajectory
sprintf('Horizontal error = %0.5g , Spherical error = %0.5g',sqrt(sum((x_h(1:2,end)).^2)), sqrt(sum((x_h(1:3,end)).^2)))


%% View the result 
disp('Views the data')
view_data;

