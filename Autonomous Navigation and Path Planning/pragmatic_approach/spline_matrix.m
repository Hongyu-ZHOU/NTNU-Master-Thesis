function M = spline_matrix(n)

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