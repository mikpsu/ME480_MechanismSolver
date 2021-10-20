%% ME480 Mechanism Solver
%Author: Michael Pagan
clc, clear, close all

%% 4 Bar Pin Joint Mechanism Parameters
% Link lengths
a = 3; %input (crank) length
b = 10; %coupler length
c = 6; %output length
d = sqrt(1.5^2+12^2); %ground length
units = 'in'; %units of above link lengths

% transform between local and global coordinates
theta1 = atand(12/1.5); %ccw angle from local +x axis to global X axis
     %local +x points from crank ground to ouput ground (O2 to O4)  
     
% point of interest P on linkage
P_link = 'c'; %define which link the point is on: 'a' 'b' or 'c'
delta = 0; %fixed angle between selected link vector and point vector on 
                %link. e.g. angle between AB and AP. cw = negative
p = c; %distance from link pin joint to desired point P (e.g. AP)

% position of mechanism shown in figure
theta2_fig_g = 130; %approximate theta 2 for determining configuration of 
                    %figure. Global system. Must be between 0-360 deg.
theta3_fig_g = 270; %approximate theta 3 for determining configuration of 
                    %figure. Global system. Must be between 0-360 deg. 
theta2_fig_l = theta2_fig_g + theta1; %DO NOT EDIT                 

% given speed and acceleration of crank
omega2 = 120*pi/30; %input velocity
alpha2 = 0;

%Custom input range for crank in local coordinates
override_togs = 0; %change to 1 to use custom range
custom_input = theta2_fig_l; %specify custom range LOCAL coordinates

       
%No edits must be made below this header%    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   _  _     ____          _____    _____ _____ _   _ 
%  | || |   |  _ \   /\   |  __ \  |  __ \_   _| \ | |
%  | || |_  | |_) | /  \  | |__) | | |__) || | |  \| |
%  |__   _| |  _ < / /\ \ |  _  /  |  ___/ | | | . ` |
%     | |   | |_) / ____ \| | \ \  | |    _| |_| |\  |
%     |_|   |____/_/    \_\_|  \_\ |_|   |_____|_| \_|
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                                   
%% Toggle Positions
% Grashof determination
links = [a,b,c,d];
grash = 0; %flag for grashof
if (max(links)+ min(links)) <= sum(links)-(max(links)+ min(links))
    disp('GRASHOF MECHANISM')
    grash = 1;
else
    disp('NON-GRASHOF MECHANISM')
end

% Determine toggle points
%Flags for number of toggles
Onetog = 0;
Twotog = 0;
Notog = 0;

% Test for NaN case
tog1_inards = (a.^2+d.^2-b.^2-c.^2)./(2*a*d)+b*c./(a*d);
tog2_inards = (a.^2+d.^2-b.^2-c.^2)./(2*a*d)-b*c./(a*d);

% Calculate the two toggle positions, in local coordinates
if xor((abs(tog1_inards) > 1), (abs(tog2_inards) > 1))
    %checks that ONLY one of the toggle equations is valid
    Onetog = 1; %flag that we have opposite/equal togs
    disp('EQUAL AND OPPOSITE TOGGLE POSITIONS')
    if abs(tog1_inards) < 1 %plus case is valid
        theta_tog1_l = (acosd(tog1_inards)); %plus
        theta_tog2_l = -theta_tog1_l;

    else %minus case is valid
        theta_tog2_l = (acosd(tog2_inards)); % minus
        theta_tog1_l = -theta_tog2_l;
    end
elseif (abs(tog1_inards) > 1) && (abs(tog2_inards) > 1) 
    %checks that NEITHER toggle equations are valid
    disp('NO TOGGLE POSITIONS')
    Notog = 1;
else %both toggle equations must be valid
    disp('TWO UNIQUE TOGGLE POSITIONS')
    Twotog = 1;
    theta_tog1_l = (acosd(tog1_inards)); % plus
    theta_tog2_l = (acosd(tog2_inards)); % minus
end

%% Create input vector for current configuration
%determine if approximate theta 2 is between cw or ccw range of togs
if not(Notog)
    %create vector with original toggle points
    togs_l = [theta_tog1_l, theta_tog2_l];
    % create vector with positive toggle points
    togs_l_pos = togs_l + 360.*(togs_l < 0);
    % convert to global
    togs_g = togs_l_pos - theta1;
    
    if theta2_fig_l < max(togs_l_pos) && theta2_fig_g > min(togs_l_pos)
        toggle_dir = 1; %in this case the configuration lies in the ccw range
                        %of the toggle positions
    else
        toggle_dir = 0; %else configuration must be in the cw range
    end
end

%case for opposite and equal toggle angles. abs will be 0 to 180 deg
if override_togs > 0
    theta2l = custom_input;
    theta2g = theta2l - theta1;
else
    if Onetog
        % case for opposite and equal toggle angles
        if toggle_dir < 1 %from positive tog through +x axis to negative tog
            theta2l = flip(min(togs_l):0.5:max(togs_l)); %local
            theta2l = theta2l + 360.*(theta2l < 0);
        else % from positive tog through -x axis to negative tog
            theta2l = min(togs_l_pos):0.5:max(togs_l_pos); %local
        end

    elseif Twotog
        %case for two unique toggle points. togs will be [0 , 180] deg local
        % direction irrelevant bc range does not cross x axis
        theta2l = min(togs_l):0.5:max(togs_l); %local

    elseif Notog
        %case for no toggle points. Input angle theta 2 can go 360 deg
        theta2g = 0:0.5:360;
        theta2l = theta2g + theta1;
    end
end

% convert local input range to global
if not(Notog)
    theta2g = theta2l - theta1;
    theta2g = theta2g + 360.*(theta2g < 0);
end

% Make all negative input angles positive
theta2l = theta2l + 360.*(theta2l < 0);

%% Calculated Parameters
k1 = d./a;
k2 = d./c;
k3 = (a.^2-b.^2+c.^2+d.^2)./(2*a*c);
k4 = d./b;
k5 = (c.^2-d.^2-a.^2-b.^2)./(2*a*b);

A = cosd(theta2l)-k1-k2.*cosd(theta2l)+k3;
B = -2.*sind(theta2l);
C = k1-(k2+1).*cosd(theta2l)+k3;
D = cosd(theta2l)-k1+k4.*cosd(theta2l)+k5;
E = -2.*sind(theta2l);
F = k1 + (k4-1).*cosd(theta2l)+k5;

%% Output Angle Theta 4

%OPEN CONFIGURATION
%calculate the open configuration output angle. "Minus" eq
%find and remove imaginary input arguments
Q_m4 = (-B-sqrt(B.^2-4.*A.*C));
Q_m4(imag(Q_m4) ~= 0) = NaN;
%calculate the angle
theta4open_l = (2.*atan2d(Q_m4, 2.*A)); %local,
%convert to global. We only report output angles in global.
theta4open_g = theta4open_l - theta1;
%find and correct large and/or negative angles
for j = 1:2 %loop through twice for angles < -360
    theta4open_g = theta4open_g + 360.*(theta4open_g < 0);
end

%CLOSED CONFIGURATION
%calculate the closed configuration output angle. "Plus" eq
%find and remove imaginary input arguments
Q_p4 = (-B+sqrt(B.^2-4.*A.*C));
Q_p4(imag(Q_p4) ~= 0) = NaN;
%calculate the angle
theta4closed_l = (2.*atan2d(Q_p4,2.*A));%local,
%convert to global
theta4closed_g = theta4closed_l - theta1;
%find and correct large and/or negative angles
for j = 1:2
    theta4closed_g = theta4closed_g + 360.*(theta4closed_g < 0);
end

%DEPICTED CONFIGURATION
%calculate D,E,F for the current position
A_fig = cosd(theta2_fig_l)-k1-k2.*cosd(theta2_fig_l)+k3;
B_fig = -2.*sind(theta2_fig_l);
C_fig = k1-(k2+1).*cosd(theta2_fig_l)+k3;

%calculate closed theta 3 for current position
theta4closed_fig_l = (2.*atan2d(-B_fig+sqrt(B_fig.^2-4.*A_fig.*C_fig),...
                                    2.*A_fig)); %local, plus
%convert to global                  
theta4closed_fig_g = theta4closed_fig_l - theta1; 

%calculate open theta 3 for current position
theta4open_fig_l = 2.*atan2d(-B_fig-sqrt(B_fig.^2-4.*A_fig.*C_fig),...
                                    2.*A_fig); %local, minus
%convert to global                    
theta4open_fig_g = theta4open_fig_l - theta1;

%make sure angles are 0-360
for k = 1:2
    theta4open_fig_g = theta4open_fig_g + 360.*(theta4open_fig_g < 0);
    theta4closed_fig_g = theta4closed_fig_g + 360.*(theta4closed_fig_g < 0);
end

%% Coupler Angle Theta 3

%OPEN CONFIGURATION
%calculate the open configuration coupler angle. "Minus case"
%find and remove imaginary input arguments
Q_m3 = (-E-sqrt(E.^2-4.*D.*F));
Q_m3(imag(Q_m3) ~= 0) = NaN;
%calculate the angle
theta3open_l = (2.*atan2d(Q_m3,2.*D));%local, 
%convert to global
theta3open_g = theta3open_l - theta1;
%find and correct negative angles
for j = 1:2
    theta3open_g = theta3open_g + 360.*(theta3open_g < 0);
end

%CLOSED CONFIGURATION
%calculate the closed configuration coupler angle. "Plus case:"
%find and remove imaginary input arguments
Q_p3 = (-E+sqrt(E.^2-4.*D.*F));
Q_p3(imag(Q_p3) ~= 0) = NaN;
%calculate the angle
theta3closed_l = (2.*atan2d(Q_p3, 2.*D)); %local, plus
%convert to global
theta3closed_g = theta3closed_l - theta1; %
for j = 1:2
    theta3closed_g = theta3closed_g + 360.*(theta3closed_g < 0);
end

%DEPICTED CONFIGURATION
%calculate current coupler angle in given figure based on estimated theta2
%calculate D,E,F for the current position
D_fig = cosd(theta2_fig_l)-k1+k4.*cosd(theta2_fig_l)+k5;
E_fig = -2.*sind(theta2_fig_l);
F_fig = k1 + (k4-1).*cosd(theta2_fig_l)+k5;

%calculate closed theta 3 for current position
theta3closed_fig_l = (2.*atan2d((-E_fig+sqrt(E_fig.^2-4.*D_fig.*F_fig)),...
                      2.*D_fig)); %local, plus
%convert to global                  
theta3closed_fig_g = theta3closed_fig_l - theta1; 

%calculate open theta 3 for current position
theta3open_fig_l = (2.*atan2d((-E_fig-sqrt(E_fig.^2-4.*D_fig.*F_fig)),...
                        2.*D_fig));%local, 
%convert to global                    
theta3open_fig_g = theta3open_fig_l - theta1;

%make sure angles are 0-360
for k = 1:2
    theta3open_fig_g = theta3open_fig_g + 360.*(theta3open_fig_g < 0);
    theta3closed_fig_g = theta3closed_fig_g + 360.*(theta3closed_fig_g < 0);
end

%% Determine Configuration of Depiction in Figure
%check if open or closed coupler angle is closer to visually-estimated
    %coupler angle in figure. Comparison of absolute differences
if abs(theta3open_fig_g-theta3_fig_g) > abs(theta3closed_fig_g-theta3_fig_g)
    disp('FIGURE DEPICTS THE CLOSED CONFIGURATION')
    % flags for open/closed
    closed = 1;
    open = 0;
    
    theta3current_fig_g = theta3closed_fig_g;
    theta4current_fig_g = theta4closed_fig_g;
    
    theta3g = theta3closed_g;
    theta4g = theta4closed_g;
else
    disp('FIGURE DEPICTS THE OPEN CONFIGURATION')
    % flags for open/closed
    open = 1;
    closed = 0; 
    
    theta3current_fig_g = theta3open_fig_g;
    theta4current_fig_g = theta4open_fig_g;
    
    theta3g = theta3open_g;
    theta4g = theta4open_g;
end

%ensure the angles are (0-360)
for k = 1:2
    theta3current_fig_g = theta3current_fig_g + 360.*(theta3current_fig_g < 0);
    theta3current_fig_g = theta3current_fig_g - 360.*(theta3current_fig_g > 360);
    theta4current_fig_g = theta4current_fig_g + 360.*(theta4current_fig_g < 0);
    theta4current_fig_g = theta4current_fig_g - 360.*(theta4current_fig_g > 360);
end

%% Plot Coupler and Output Angles
figure(1)
hold on
plot((theta2g), theta4g,'.')
xlabel('Input Angle (\theta_2) [deg]')
ylabel('Output Angle (\theta_4) [deg]')
plot(theta2_fig_g, theta4current_fig_g, 'x','linewidth',3)
legend( '', 'Position in Figure')
if open
    title('Output Angle vs. Input Angle (Open, Global)')
else
    title('Output Angle vs. Input Angle (Closed, Global)')
end

figure(2)
hold on
plot(theta2g, theta3g, '.');
xlabel('Input Angle (\theta_2) [deg]')
ylabel('Coupler Angle (\theta_3) [deg]')
plot(theta2_fig_g, theta3current_fig_g, 'x','linewidth',3)
legend('','Position in Figure')
if open
    title('Coupler Angle vs. Input Angle in (Open, Global)')
else
    title('Coupler Angle vs. Input Angle in (Closed, Global)')
end

%% Point P on Coupler
%find the vector P in global space. 
if P_link == 'a' %point P is on link a (input). Pinned at O2
    %find angle of  O2P
    thetaP_g = theta2g + delta;
    
    % P-> = AP-> relative to O2
    P = p.*cosd(thetaP_g) + 1i.*p.*sind(thetaP_g);  
    
    %current position of point P
    thetaPcurrent_g = theta2_fig_g + delta;
    P_current = p.*cosd(thetaPcurrent_g) + 1i.*p.*sind(thetaPcurrent_g);  
    
elseif P_link == 'b' %point P is on link b (coupler). Pinned at A
    %find angle of AP
    thetaP_g = theta3g + delta;
    
    %P-> = AB-> + BP-> relative to O2
    P = a.*cosd(theta2g) + p.*cosd(thetaP_g) + ...
        1i.*(a.*sind(theta2g) + p.*sind(thetaP_g));
    
    %current position of point P
    thetaPcurrent_g = theta3current_fig_g + delta;
    P_current = a.*cosd(theta2_fig_g) + p.*cosd(thetaPcurrent_g) + ...
        1i.*(a.*sind(theta2_fig_g) + p.*sind(thetaPcurrent_g));
    
elseif P_link == 'c' %point P is on link c (output)
    %find angle O4P
    thetaP_g = theta4g + delta;
                                       
    
    % P-> = O2P-> relative to O4
    P = p.*cosd((thetaP_g)) + 1i.*p.*sind((thetaP_g));

    %current position of point P relative to O4
    thetaPcurrent_g = theta4current_fig_g + delta;
    P_current = p.*cosd(thetaPcurrent_g) + 1i.*p.*sind(thetaPcurrent_g); 
    
else %point P link incorrectly defined
    disp('Incorrect definition of link for point P. Check  var ''P_link''')
end

Px = real(P);
Py = imag(P);
Px_current = real(P_current);
Py_current = imag(P_current);

%% Plot Position on Coupler

figure(4)
plot(Px, Py, '.')
xlabel({"X Position [" + units + "["})
ylabel("Y Position [" + units + "]")
daspect([1 1 1]);

if open
    title('Position of Point P (Open)')
else
    title('Position of Point P (Closed)')
end

hold on
%plot current position of P
plot(Px_current, Py_current, 'x','linewidth',2)

%plot and label the origin
plot(0,0,'ks','linewidth', 4)
if P_link == 'b' || P_link == 'a' %O2 is at the origin
    text(.05.*range([Px]),0, {'O_2'})
elseif P_link == 'c' %O4 is at origin
    text(.05.*range([Px]),0, {'O_4'})
end

%% Velocity Analysis
omega3 = a.*omega2./b .* (sind(theta4g-theta2g)./sind(theta3g-...
    theta4g));
omega4 = a.*omega2./c .* (sind(-theta3g+theta2g)./sind(theta4g-...
    theta3g));

% filter out very high velocities
omega3(abs(theta3g-theta4g) < 0.1) = NaN;
omega4(abs(theta3g-theta4g) < 0.1) = NaN;

% velocity of A (crank end), is tangential to crank arc
VA_g = a.*omega2.*(-sind(theta2g)+1i.*cosd(theta2g));

% velocity of B relative to A
VBA_g = b.*omega3 .*(-sind(theta3g)+1i.*cosd(theta3g));

% velocity of B, is tangential to output arc
VB_g = c.*omega4 .*(-sind(theta4g)+1i.*cosd(theta4g));

% velocity of P relative to A
VPA_g = p.*omega3 .* -sind(theta3g + delta) + 1i.*p.*omega3 ...
    .* cosd(theta3g+delta);
% velocity of P
VP_g = VA_g + VPA_g;

% velocity magnitudes
VA_mag = abs(VA_g);
VB_mag = abs(VB_g);
VP_mag = abs(VP_g);

%% Plot velocity
figure(5)
plot(theta2g, VP_mag, '.');
xlabel('Input Angle (\theta_2) [deg]')
ylabel("|V_P| [" + units +"/s]")

if open
    title('Point P Velocity vs Input Angle (Open)')
else
    title('Point P Velocity vs Input Angle (Closed)')
end

%% Acceleration Analysis
A_a = c*sind(theta4g);
B_a = b*sind(theta3g);
C_a = a*alpha2.*sind(theta2g)+a*omega2.^2.*cosd(theta2g)+b*omega3.^2 ...
    .*cosd(theta3g) - c*omega4.^2 .*cosd(theta4g);
D_a = c*cosd(theta4g);
E_a = b*cosd(theta3g);
F_a = a*alpha2.*cosd(theta2g)- a*omega2.^2.*sind(theta2g)-b*omega3.^2 ...
    .*sind(theta3g) + c*omega4.^2 .*sind(theta4g);

alpha3 = (C_a.*D_a - A_a.*F_a)./(A_a.*E_a - B_a.*D_a);
alpha4 = (C_a.*E_a - B_a.*F_a)./(A_a.*E_a - B_a.*D_a);

%% Print Relevant Values
%print values only if evaluating for one theta 2 input
if length(theta2g) == 1
    %define output vectors
    links = ["Crank (a)", "Coupler (b)", "Output (c)"]';
    thetas = [theta2g, theta3g, theta4g]';
    omegas = [omega2, omega3, omega4]';
    alphas = [alpha2 , alpha3, alpha4]';
    %create and display the table
    mechanism_state = table;
    mechanism_state.Link = links;
    mechanism_state.Angle = thetas;
    mechanism_state.Speed = omegas;
    mechanism_state.Acceleration = alphas;
    mechanism_state.Properties.VariableNames{2} = 'Angle [deg]';
    mechanism_state.Properties.VariableNames{3} = 'Speed [rad/s]';
    mechanism_state.Properties.VariableNames{4} = 'Acceleration [rad/s^2]';
    disp(mechanism_state)
end

%% troubleshooting inputs

