function output = calculate_bezier(CP,P_b)
% Calculates the Bezier curve, its deriavites (up to order 3), curvature
% and rate of change in curvature.
%
% Input:
% - CP: Control points
% - P_b: blending functions
%
% Output:
% - output: Struct of given outputs
%
% output = calculate_bezier(CP,P_b) the Bezier curve, its deriavites 
% (up to order 3), curvature and rate of change in curvature. 
%
% Magnus Knaedal

output = struct();

output.B_matrix = P_b.B_blending*CP;
output.dot_B_matrix = P_b.dot_B_blending*CP;
output.ddot_B_matrix = P_b.ddot_B_blending*CP;
output.dddot_B_matrix = P_b.dddot_B_blending*CP;

output.direction = rad2deg(atan2(output.dot_B_matrix(:,2),output.dot_B_matrix(:,1)));

output.K = curvature(CP, P_b.dot_B_blending, P_b.ddot_B_blending);
output.dot_K = derivative_curvature(output.dot_B_matrix, output.ddot_B_matrix, output.dddot_B_matrix);
