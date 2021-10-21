%% ME480 Mechanism Solver
%Author: Michael Pagan
clc, clear, close all

%% Crank-Slider OR Slider-Crank Mechanism Parameters
% link lengths
a = 10.6; %crank-slider crank
b = 22.7; %crank-slider coupler
c = 0; %crank-slider offset: normal distance from local axis to global axis
           % the slider moves along the local axis
units = "cm"; %units of above link lengths

% define problem type: must choose one
cs = 0; % >0 if crank-slider problem
sc = 1; % >0 if slider-crank problem
                
% crank-slider inputs
theta2 = 20; %input crank angle
omega2 = 100*2*pi/60; %input crank velocity
alpha2 = 0; %input crank acceleration

% slider-crank inputs
d = -28;
d_dot = 10; %input slider velocity
d_ddot = 0; %input slider acceleration

% depiction in figure
theta2_fig = 130; %approximate theta 2 for determining configuration of 
                    %figure. Global system. Must be between 0-360 deg.                  
theta3_fig = 20; %approximate theta 3 of crank slider. 
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
%% Crank Slider Position Analysis
if cs
    %Determine configuration of given figure
    theta3_closed_fig = asind((a*sind(theta2_fig)-c)/b);
    theta3_open_fig = asind(-((a*sind(theta2_fig)-c)/b))+180; 

    if abs(theta3_open_fig-theta3_fig) > abs(theta3_closed_fig-theta3_fig)
        disp('FIGURE DEPICTS THE CLOSED CONFIGURATION')
        % flags for open/closed
        closed = 1;
        open = 0;

        theta3current_fig = theta3_closed_fig;
        
        theta3 = asind((a*sind(theta2)-c)/b);
        d = a*cosd(theta2)-b*cosd(theta3);
    else 
        disp('FIGURE DEPICTS THE OPEN CONFIGURATION')
        % flags for open/closed
        closed = 0;
        open = 1;

        theta3current_fig = theta3_closed_fig; 
        
        theta3 = asind(-((a*sind(theta2)-c)/b))+180;
        d = a*cosd(theta2)-b*cosd(theta3);
    end
    
    % define theta2 as a full rotation
    theta2 = 0:0.5:360;
end

%% Slider Crank Position Analysis
if sc
    % calcualte theta2
    K1 = a^2-b^2+c^2+d^2;
    K2 = -2*a*c;
    K3 = -2*a*d;
    A = K1-K3;
    B = 2*K2;
    C = K1+K3;
    
    theta2_branch(1) = 2*atan2d(-B+sqrt(B^2-4*A*C), 2*A); % branch 1
    theta2_branch(2) = 2*atan2d(-B-sqrt(B^2-4*A*C), 2*A); % branch 2
    % Make all negative input angles positive
    theta2_branch = theta2_branch + 360.*(theta2_branch < 0);
    
    % determine which branch we want to analyze
    if abs(theta2_fig-theta2_branch(1)) > abs(theta2_fig-theta2_branch(2))
        disp('FIGURE DEPICTS BRANCH 2')
        % flags for branch 1 or 2
        branch1 = 0;
        branch2 = 1;

        theta2 = theta2_branch(2);
    else 
        disp('FIGURE DEPICTS BRANCH 1')
        % flags for branch 1 or 2
        branch1 = 1;
        branch2 = 0;

        theta2 = theta2_branch(1);
    end
    
    %Determine configuration of given figure
    theta3_closed = asind((a*sind(theta2)-c)/b);
    theta3_open = asind(-((a*sind(theta2)-c)/b))+180; 
    
    if abs(theta3_open-theta3_fig) > abs(theta3_closed-theta3_fig)
        disp('FIGURE DEPICTS THE CLOSED CONFIGURATION')
        % flags for open/closed
        closed = 1;
        open = 0;

        theta3 = theta3_closed;
    else 
        disp('FIGURE DEPICTS THE OPEN CONFIGURATION')
        % flags for open/closed
        closed = 0;
        open = 1;

        theta3 = theta3_open; 
    end
end

%% Velocity Analysis
if sc
    omega2 = d_dot.*cosd(theta3) ./ (a*(cosd(theta2).*sind(theta3)-sind(theta2).*cosd(theta3)));
end
omega3= a/b .* cosd(theta2)./cosd(theta3) .* omega2;
d_dot = -a*omega2.*sind(theta2)+b*omega3.*sind(theta3);

%% Acceleration Analysis
if sc
    alpha2 = (a*omega2.^2*(cosd(theta2).*cosd(theta3)+sind(theta2) ...
        .*sind(theta3)) - b*omega3.^2 + d_ddot*cosd(theta3)) ./ ...
        (a*(cosd(theta2).*sind(theta3)-sind(theta2).*cosd(theta3)));
    
    alpha3 = (a.*alpha2.*cosd(theta2)-a.*omega2.^2.*sind(theta2)+ ...
        b.*omega3.^2.*sind(theta3)) ./ b.*cosd(theta3);
end
   
if cs
    alpha3 = (a*alpha2*cosd(theta2)-a*omega2.^2.*sind(theta2)+ ...
        b*omega3.^2.*sind(theta3)) ./ b*cosd(theta3);

    d_ddot = -a*alpha2*sind(theta2) - a*omega2.^2.*cosd(theta2) + ...
        b*alpha3*sind(theta3) + b*omega3.^2.*cosd(theta3);
end

%% Plot Position, Velocity, Acceleration
%Plot the position of the slider
figure(1)
plot(d, c+zeros(length(d)),'.')
title('Slider Position')
xlabel("X Position [" + units + "]")
ylabel("Y Position ["+units+"]") 
hold on
%Assuming pinned joint of crank is origin
plot(0,0,'ks','linewidth', 5)
daspect([1 1 1]);
text(.05*range(d),0, {'O_2'})

% velocity of the slider
figure(2)
plot(theta2, d_dot, '.');
title('Slider Velocity vs Input Angle (Global)')
xlabel('Input Angle (\theta_2) [deg]')
ylabel("V_d ["+units+"/s]")

% velocity of the crank

% velocity of the coupler
%% Print Relevant Values
%print values only if evaluating for one theta 2 input
if length(theta2) == 1
    %define output vectors
    links = ["Crank (a)", "Coupler (b)", "Slider (d)"]';
    thetas = [theta2, theta3, d]';
    omegas = [omega2, omega3, d_dot]';
    alphas = [alpha2 , alpha3, d_ddot]';
    units = ["rad", "rad", "length"]';
    %create and display the table
    mechanism_state = table;
    mechanism_state.Link = links;
    mechanism_state.Angle = thetas;
    mechanism_state.Speed = omegas;
    mechanism_state.Acceleration = alphas;
    mechanism_state.Units = units;
    mechanism_state.Properties.VariableNames{2} = 'Position';
    mechanism_state.Properties.VariableNames{3} = 'Speed';
    mechanism_state.Properties.VariableNames{4} = 'Acceleration';
    mechanism_state.Properties.VariableNames{5} = 'Units';
    disp(mechanism_state)
end
