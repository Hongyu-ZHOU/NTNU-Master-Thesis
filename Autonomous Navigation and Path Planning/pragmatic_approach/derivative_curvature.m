function dot_K = derivative_curvature(dot_B, ddot_B, dddot_B)
% Calculates the curvator of the bezier curve
%
% Input:
% - P = control points
% - dot_B, ddot_B: blending functions
%
% Magnus Knaedal 28.12.2019
%
% TODO: Take in control points and not precalculated, for consistency.
%%


dot_K = zeros(1, length(dot_B)); % change to size_t

for i = 1:length(dot_B) 
    d_x = dot_B(i,1);
    dd_x =  ddot_B(i,1);
    ddd_x = dddot_B(i,1);

    d_y = dot_B(i,2);
    dd_y =  ddot_B(i,2);
    ddd_y = dddot_B(i,2);
    
    
    dot_K(i) = ( (ddd_y * d_x - ddd_x * d_y) / (d_x^2 + d_y^2)^(3/2) ) ...
          - (3 * (d_x * dd_y - dd_x * d_y) * (2 * d_x * dd_x + 2 * d_y * dd_y) ) / (2 * (d_x^2 + d_y^2)^(5/2));
    
    
    %dot_K(i) = ((ddd_y * d_x - ddd_x *  d_y )/((d_x^2 + d_y^2)^(3/2))) - ((3 * (d_x *  dd_y - dd_x * d_y) * (2 * d_x * dd_x + 2 * d_y * dd_y)) / (2 * (d_x^2 + d_y^2)^(5/2)));
    
    % With absolute value
    %dot_K(i) = ((ddd_y * d_x - ddd_x *  d_y ) / ((d_x^2 + d_y^2)^(3/2))) - (((3 * (d_x * dd_y - dd_x * d_y) * (2 * d_x * dd_x + 2 * d_y * dd_y))) / (2 * (d_x^2 + d_y^2)^(5/2)));
    
    %dot_K(i) = ((ddd_y  * d_x - ddd_x *  d_y) * (d_x *  dd_y - dd_x *  dd_y)) / (( d_x^2 + d_y^2)^(3/2) *  abs(d_x *  dd_y - d_y *  dd_x)) - (3 * (2 * d_x * dd_x + 2 * d_y * dd_y) * abs(d_x * dd_y - d_y * dd_x))/(2 * (d_x^2 + d_y^2)^(5/2));
    if isnan(dot_K(i))
        dot_K(i) = 0;
    end

end

end