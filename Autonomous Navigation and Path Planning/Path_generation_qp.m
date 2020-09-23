function [CP,Bezier_con,Q] = Path_generation_qp(pos,k,CP_prev, pd_initial,Q)
% Path_generation 
% Script for search-space approach.
% Magnus Knaedal 30.08.2019
% Modified by Hongyu Zhou 19.04.2020

addpath(genpath(pwd));

%% Parameters
h = 0.001; % stepsize
theta = 0:h:1;
n = 8; % # control points
objective = 1; % Objective function

% Tuning variables
order = 8; % not used now
zeta = 0.5; % wall distance
k_max = 0.005; % max curvature
posib = 20; % size of search space

% Define waypoints
WP =[pos(k-1,2),pos(k-1,1) ; pos(k,2),pos(k,1)];


% Initialize parameters
if k==2
    psi_current = -pi/2;
%     Q = 0; % Distance
else
    psi_current = atan2(pos(k-1,2) - pos(k-2,2), ...
                     pos(k-1,1) - pos(k-2,1));
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
   
    CP = init_cp(WP_current, psi_current, WP_next, psi_next, k, CP_prev);
    CP_opt = quadratic_programming(CP, n, zeta, psi_next);

    Bezier = calculate_bezier(CP,P_b); % Calculate Bezier
    Bezier_con = [Bezier.B_matrix(:,2) Bezier.B_matrix(:,1) Bezier.dot_B_matrix(:,2) Bezier.dot_B_matrix(:,1)...
                  Bezier.ddot_B_matrix(:,2) Bezier.ddot_B_matrix(:,1)...
                  Bezier.dddot_B_matrix(:,2) Bezier.dddot_B_matrix(:,1)];

    q = distance(CP, P_b.dot_B_blending);
    Q = Q + q;
           

end

end

