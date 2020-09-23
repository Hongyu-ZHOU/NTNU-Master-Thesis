function K = curvature(P, dot_B, ddot_B)
% Calculates the curvator of the bezier curve
%
% Input:
% - P = control points
% - dot_B, ddot_B: blending functions
%
% Magnus Knaedal 28.12.2019
%%

x_p = P(:,1);
y_p = P(:,2);

K = zeros(1, length(dot_B));

for i = 1:length(dot_B)
    K(i) = abs((dot_B(i,:)*x_p * ddot_B(i,:)*y_p - ddot_B(i,:)*x_p * dot_B(i,:)*y_p ))/ ...
    ((dot_B(i,:)*x_p)^2 + (dot_B(i,:)*y_p)^2)^(3/2);
    if isnan(K(i))
        K(i) = 0;
    end

end

end