function CP = init_cp(WP_current, psi_prev, WP_next, psi_next, i, CP_prev)
% Calculate the first 4 control points for current segment and initailize
% the matrix of control points.
%
% Input:
% - WP_current: current WP
% - psi_prev: previous heading
% - WP_next: next WP
% - psi_next: next heading
% - i: Segment number.
% - CP_prev: Control points from previous segment
%
% Output:
% - CP: Initial placement of control points for current segment.
%
% CP = init_cp(WP_current, WP_next, psi_next, i, CP_prev) calculates
% control points P0 to P3, and initilize control point list.
%
% Magnus Knaedal 

% scaling factor for initializing points
delta_max = norm(WP_next - WP_current)/2; 

if i == 2 % If first segment
        
    CP = [WP_current;
          WP_current + delta_max * [cos(psi_prev) sin(psi_prev)]*1/6;        
          WP_current + delta_max * [cos(psi_prev) sin(psi_prev)]*2/6;
          WP_current + delta_max * [cos(psi_prev) sin(psi_prev)];
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]; 
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]*2/6;         
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]*1/6; 
          WP_next];
else
    
    % Formulate and solve system of linear equation Ax = B
    A = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         -2 0 1 0 0 0;
         0 -2 0 1 0 0;
         3 0 -3 0 1 0;
         0 3 0 -3 0 1];

     B = [2*CP_prev(8,1)-CP_prev(7,1);
          2*CP_prev(8,2)-CP_prev(7,2);
          -2*CP_prev(7,1)+CP_prev(6,1);
          -2*CP_prev(7,2)+CP_prev(6,2);
          2*CP_prev(8,1)-3*CP_prev(7,1)+3*CP_prev(6,1)-CP_prev(5,1);
          2*CP_prev(8,2)-3*CP_prev(7,2)+3*CP_prev(6,2)-CP_prev(5,2);
          ];

    x = mldivide(A,B);
    % Note: P4, P5, P6 is not necassary to calculate, since found by
    % optimization algorithm.
    CP = [WP_current;
          [x(1) x(2)];
          [x(3) x(4)];
          [x(5) x(6)];
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]; 
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]*2/6;         
          WP_next - delta_max * [cos(psi_next) sin(psi_next)]*1/6; 
          WP_next];
end

end