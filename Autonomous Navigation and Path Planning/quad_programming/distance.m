function Q = distance(P, dot_B)
% Calculates the arc length of the bezier curve
%
% Input:
% - P = control points
% - dot_B: blending functions
%
% Magnus Knaedal 28.12.2019
%%
x_p = P(:,1);
y_p = P(:,2);

S = zeros(1, length(dot_B));

for i = 1:length(dot_B)
    S(i) = sqrt( (dot_B(i,:)*x_p)^2 + (dot_B(i,:)*y_p)^2);
    
    if isnan(S(i))
        S(i) = 0;
    end

end

% Trapezoid approximation of integral
h = 0.001; % TODO: add as input
theta = 0:h:1;
Q = trapz(theta, S);
end