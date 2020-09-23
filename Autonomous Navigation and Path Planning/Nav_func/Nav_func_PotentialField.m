function [psi_p,Psi_p] = Nav_func_PotentialField(Mode,X,Y,Nx,Ny,ObstacleIdx,TargetIdx,r_world,r_o,r)
% PotentialField calculates the porential at discete points using navigation function
% 
% Hongyu Zhou
% 


Psi_p = zeros(Ny,Nx);
[No,~] = size(ObstacleIdx);
dist = 0;

% tuning parameter
switch Mode
    case {1,2}
        kappa = 1.6;  % Mode 1-2
    case {3,4,5}
        kappa = 2.2;  % Mode 3-5
    case {6,7}
        kappa = 3.8;  % Mode 6-7
    case 8
        kappa = 4.8;  % Mode 8
    case 9
        kappa = 5.4;  % Mode 9
end

for i = 1:Ny
    for j = 1:Nx
    gamma = norm([X(i,j)-X(TargetIdx(1),TargetIdx(2)),Y(i,j)-Y(TargetIdx(1),TargetIdx(2))])^2;
    beta = r_world^2 - norm([X(i,j),Y(i,j)])^2 - r^2;
    
        if beta <= 0
            Psi_p(i,j) = 1;
            continue;
        end
    
        for k = 1:No
            dist = norm([X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)),...
                                  Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))])^2....
                              - (r+r_o)^2;
            if dist <= 0
                beta = 0;
                break;                
            end
            
            beta = beta * ( norm([X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)),...
                                  Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))])^2 );
        end
     Psi_p(i,j) = gamma/(gamma^kappa+beta)^(1/kappa);
    end
end

psi_p = reshape(Psi_p,[Ny*Nx,1]);

end

