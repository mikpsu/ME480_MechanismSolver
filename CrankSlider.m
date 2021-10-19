%% ME480 Mechanism Solver
%Author: Michael Pagan
clc, clear, close all

%% 4 Bar Pin Joint Mechanism Parameters
% Link lengths
a = 0.785; %input (crank) length
b = 0.356; %coupler length
c = 0.95; %output length
d = 0.544; %ground length

% transform between local and global coordinates
theta1 = (0); %ccw angle from local +x axis to global X axis
     %local +x points from crank ground to ouput ground (O2 to O4)
     
% point of interest P on linkage
P_link = 'b'; %define which link the point is on: 'a' 'b' or 'c'
p = 1.09; %distance from link pin joint to desired point P (e.g. AP)
delta = 0; %fixed angle between selected link vector and point vector on 
                %link. e.g. angle between AB and AP. cw = negative
                
% position of mechanism shown in figure
theta2_fig_g = 100; %approximate theta 2 for determining configuration of 
                    %figure. Global system. Must be between 0-360 deg.
theta2_fig_l = theta2_fig_g + theta1;                    
theta3_fig_g = 20; %approximate theta 3 for determining configuration of 
                    %figure. Global system. Must be between 0-360 deg.
omega_2 = 100*2*pi/60; %input velocity

%Custom input range for crank in local coordinates
override_togs = 1; %change to 1 to use custom range
custom_input = theta2_fig_l; %specify custom range LOCAL coordinates

%% Crank-Slider Mechanism Parameters
% link lengths
a_cs = c; %crank-slider crank
b_cs = 5.40; %crank-slider coupler
c_cs = 0; %crank-slider offset: normal distance from O2_cs to slider dof

% define crank of crank-slider
fourbarlink_equivalent = 'c';
connection = 'p'; %end point of crank-slider crank link. e.g. if 4bar
                %precedes crank-slider and 4bar output is the cs input,'c'

theta3_cs_fig_g = 315; %approximate theta 3 of crank slider. 
                    %for determining configuration of figure
                    %Global system. Must be between 0-360 deg.
                    
%No edits must be made below this header%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    _____ _____            _   _ _  __ _____ _      _____ _____  ______ 
%   / ____|  __ \     /\   | \ | | |/ // ____| |    |_   _|  __ \|  ____|
%  | |    | |__) |   /  \  |  \| | ' /| (___ | |      | | | |  | | |__   
%  | |    |  _  /   / /\ \ | . ` |  <  \___ \| |      | | | |  | |  __|  
%  | |____| | \ \  / ____ \| |\  | . \ ____) | |____ _| |_| |__| | |____ 
%   \_____|_|  \_\/_/    \_\_| \_|_|\_\_____/|______|_____|_____/|______|
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                                       
%% Determine Configuration of Crank-Slider Mechanism

% find theta3cs open and closed for theta2cs_g, compare to theta3_cs_fig_g
% using absolute method again.




%% Velocity Analysis
theta2cs_g = thetaP_g;
% theta4cs_g, theta3cs_g



% assign crank-slider input as 4bar output if applicable
if fourbar_to_crankslider
    theta2cs_g = thetaP_g;
else %crank slider input
    theta2cs_g = 0:0.5:360; %there are no toggle positions for cs
end

theta3cs_closed_g = asind((a_cs*sind(theta2cs_closed_g)-c_cs)/b_cs);
d_cs_closed_g = a_cs*cosd(theta2cs_closed_g)-b_cs*cosd(theta3cs_closed_g);

theta3cs_open_g = asind(-(a_cs*sind(theta2cs_open_g)-c_cs)/b_cs)+180;
d_cs_open_g = a_cs*cosd(theta2cs_open_g)-b_cs*cosd(theta3cs_open_g);

%slider position must be >0 for this mechanism
d_cs_open_g(d_cs_open_g<0) = NaN;
d_cs_closed_g(d_cs_closed_g<0) = NaN;
%Plot the position of the slider
figure(6)
plot(d_cs_open_g, zeros(length(d_cs_open_g)),'.')
title('Slider Position')
xlabel('X Position [units]')
ylabel('Y Position [units]')
hold on
%Assuming pinned joint of crank is origin
plot(0,0,'ks','linewidth', 5)
daspect([1 1 1]);
text(.05*range(d_cs_open_g),0, {'O_2'})

%% Velocity of slider
omega2_cs = omega_4open;

omega3_cs_open = a_cs/b_cs * cosd(theta2cs_open_g)/cosd(theta3cs_open_g)...
    *omega2_cs;
omega3_cs_closed = a_cs/b_cs * cosd(theta2cs_closed_g)/cosd(theta3cs_closed_g)...
    *omega2_cs;

d_cs_dot_open = -a_cs*omega2_cs.*sind(theta2cs_open_g)+b_cs*omega3_cs_open...
    .*sind(theta3cs_open_g);
d_cs_dot_closed = -a_cs*omega2_cs.*sind(theta2cs_closed_g)+b_cs*omega3_cs_closed...
    .*sind(theta3cs_closed_g);

figure(7)
plot(theta2g, d_cs_dot_open, '.');
title('Slider Velocity vs Input Angle (Global)')
xlabel('Input Angle (\theta_2) [deg]')
ylabel('V_C [units/s]')
