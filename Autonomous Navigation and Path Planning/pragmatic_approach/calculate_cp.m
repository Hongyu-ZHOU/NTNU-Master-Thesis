function CP = calculate_cp(WP_current, psi_current, WP_next, psi_next, k, delta_min, my, CP_prev)
% Calculate the control points for current segment.
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


if delta_min < delta_max
    delta = delta_min;
else
    delta = delta_max;
end

if k == 2 % If first segment
    CP = [WP_current;
          WP_current + delta * [cos(psi_current) sin(psi_current)]*1/my;        
          WP_current + delta * [cos(psi_current) sin(psi_current)]*2/my;
          WP_current + delta * [cos(psi_current) sin(psi_current)];
          WP_next - delta * [cos(psi_next) sin(psi_next)]; 
          WP_next - delta * [cos(psi_next) sin(psi_next)]*2/my;         
          WP_next - delta * [cos(psi_next) sin(psi_next)]*1/my; 
          WP_next];
else

A = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     -2 0 1 0 0 0;
     0 -2 0 1 0 0;
     3 0 -3 0 1 0;
     0 3 0 -3 0 1];    

b = [2*CP_prev(8,1)-CP_prev(7,1);
     2*CP_prev(8,2)-CP_prev(7,2);
     -2*CP_prev(7,1)+CP_prev(6,1);
     -2*CP_prev(7,2)+CP_prev(6,2);
     2*CP_prev(8,1)-3*CP_prev(7,1)+3*CP_prev(6,1)-CP_prev(5,1);
     2*CP_prev(8,2)-3*CP_prev(7,2)+3*CP_prev(6,2)-CP_prev(5,2);
     ];

 x = mldivide(A,b);

 CP = [WP_current;
      [x(1) x(2)];
      [x(3) x(4)];
      [x(5) x(6)];
      WP_next - delta * [cos(psi_next) sin(psi_next)]; 
      WP_next - delta * [cos(psi_next) sin(psi_next)]*2/my;         
      WP_next - delta * [cos(psi_next) sin(psi_next)]*1/my; 
      WP_next];

end

end