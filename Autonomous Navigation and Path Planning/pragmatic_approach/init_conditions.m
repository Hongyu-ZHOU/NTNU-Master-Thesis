% function [CP_prev,psi_prev, Q, c, v, colorvec] = init_conditions(psi_init)
% % Initialize variables for path-generation
% %
% % Input:
% % - psi_next: next heading
% %
% % Output:
% % - Initialized parameters to run script
% %
% % Magnus Knaedal 
% 
% CP_prev = zeros(8,2);
% Q = 0; % Distance
% psi_prev = psi_init;
% 
% 
% % Plotting
% 
% % Color vector
% colorvec = {'r','b','g','c'};
% % Counters
% c = 1; v = 1;
% end