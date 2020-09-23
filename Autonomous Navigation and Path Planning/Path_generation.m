function [CP,Bezier] = Path_generation(pos,k,CP_prev, pd_initial)
% Path_generation 
% Script for search-space approach.
% Magnus Knaedal 30.08.2019
% Modified by Hongyu Zhou 19.04.2020

addpath(genpath(pwd));

%% Parameters
h = 0.001; % stepsize
theta = 0:h:1;
n = 8; % # control points

% Tuning variables
zeta = 0.2; % wall distance
k_max = 1; % max curvature
delta_min = 0.1; % Minimum distance from P4 to P7
my = 3; % Scaling factor for distance between P4, P5, P6, and P7

% Define waypoints
WP =[pos(k-1,:) ; pos(k,:)];


% Initialize parameters
Q = 0; % Distance
if k==2
    psi_current = pi;
else
    psi_current = atan2(pos(k-1,1) - pos(k-2,1), ...
                     pos(k-1,2) - pos(k-2,2));
end

% Calculate blending functions
P_b = blending_function(n,theta);
    
for i = 1:length(WP)-1 % for each path segment
    
    WP_current = WP(i,:);
    WP_next = WP(i+1,:);
%     psi_next = atan2(WP_next(1,1) - WP_current(1,1), ...
%                      WP_next(1,2) - WP_current(1,2));
      psi_next = atan2(WP_next(1,2) - WP_current(1,2), ...
                     WP_next(1,1) - WP_current(1,1));  
    CP = calculate_cp(WP_current, psi_current, WP_next, psi_next, k, delta_min, my, CP_prev);
    

    Bezier = calculate_bezier(CP,P_b); % Calculate Bezier

    q = distance(CP, P_b.dot_B_blending);
    Q = Q + q;

    %% Plotting
%     figure(1); grid on; axis equal;
%     ff(c) = plot(Bezier.B_matrix(:,2), Bezier.B_matrix(:,1), 'Color', colorvec{1}, "LineWidth", 1.5); hold on;
%     rr(c) = plot(CP(:,2), CP(:,1), 'k.-', 'markersize', 10); hold on;
           

end

% %% Plotting
% figure(1);
% plot(WP(:,1),WP(:,2),'ok','markersize',10); hold on;
% legend([ff(1),rr(1),r(1)],'Septic $\boldmath{B}(\theta)$', 'Control polygon', 'Walls','Interpreter','latex');
% legend('-DynamicLegend','Location','Best');
% xlabel('$x$','Interpreter','latex')
% ylabel('$y$','Interpreter','latex')


end

