function b_struct = blending_function(n,theta)
% Calculates the blending functions up to (derivative) order 3 of a Bezier
% curve curve with n control points 
%
% Input: Number of control points (4, 6, 8, 10). theta = [0:h:1] where h is
% stepsize.
%
% b_struct = blending_function(n, theta), returns struct of blending
% functions up to (derivative) order 3.
%
% Magnus Knaedal 20.09.2019


% Bezier degree
n_deg = n - 1;

% Matrix containing elements.
M = zeros(n_deg+1,n_deg+1); 

% Factorial function.
for i = 0:1:n_deg
    sigma(i+1) = factorial(n_deg)/(factorial(i)*factorial(n_deg-i));  
end


for i = 0:1:n_deg
    for j = 0:1:n_deg
        if (i - j < 0)
            M(i+1, j+1) = 0;
        else
            M(i+1, j+1) = (-1)^(i-j) * sigma(i+1) * ...
                (factorial(i)/(factorial(j)*factorial(i-j)));
        end
        
    end
end

l = [];
ll = [];
lll = [];
llll = [];

for t = theta
    if n == 4
        poly = [1 t t^2 t^3];    
        dot_poly = [0 1 2*t 3*t^2];
        ddot_poly = [0 0 2 6*t];
        dddot_poly = [0 0 0 6];
    elseif n == 6        
        poly = [1 t t^2 t^3 t^4 t^5];    
        dot_poly = [0 1 2*t 3*t^2 4*t^3 5*t^4];
        ddot_poly = [0 0 2 6*t 12*t^2 20*t^3];
        dddot_poly = [0 0 0 6 24*t 60*t^2];
    elseif n == 8
        poly =       [1 t t^2 t^3    t^4    t^5     t^6     t^7];    
        dot_poly =   [0 1 2*t 3*t^2  4*t^3  5*t^4   6*t^5   7*t^6];
        ddot_poly =  [0 0 2   6*t    12*t^2 20*t^3  30*t^4  42*t^5];
        dddot_poly = [0 0 0   6      24*t   60*t^2  120*t^3 210*t^4];
    elseif n == 10
    poly = [1 t t^2 t^3 t^4 t^5 t^6 t^7 t^8 t^9];    
    dot_poly = [0 1 2*t 3*t^2 4*t^3 5*t^4 6*t^5 7*t^6 8*t^7 9*t^8];
    ddot_poly = [0 0 2 6*t 12*t^2 20*t^3 30*t^4 42*t^5 56*t^6 72*t^7];
    dddot_poly = [0 0 0 6 24*t 60*t^2 120*t^3 210*t^4 336*t^5 504*t^6];
    end
    
    % Catenate matrices.
    l = cat(1,l,poly);
    ll = cat(1,ll,dot_poly);
    lll = cat(1,lll,ddot_poly);
    llll = cat(1,llll,dddot_poly);
end

% Create struct
b_struct = struct();
b_struct.B_blending = l*M;
b_struct.dot_B_blending = ll*M;
b_struct.ddot_B_blending = lll*M;
b_struct.dddot_B_blending = llll*M;
end

