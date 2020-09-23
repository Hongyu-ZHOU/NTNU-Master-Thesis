function [psi_s,Psi_s] = Stream_func_Streamline_vortex(X,Y,Nx,Ny,C_cir,C_v,ObstacleIdx,TargetIdx,l_o,r_o,vortex_flag)
% Stream_func_Streamline calculates the streamlines 
% 
% Hongyu Zhou
% 

[No,~] = size(ObstacleIdx);
Psi_s = zeros(size(X));
r_o = l_o;

% dist_ot = zeros(1,No);
%  for kk = 1:No
%             dist_ot(kk) = norm([X(TargetIdx(1),TargetIdx(2))-X(ObstacleIdx(kk,1),ObstacleIdx(kk,2)),...
%                                Y(TargetIdx(1),TargetIdx(2))-Y(ObstacleIdx(kk,1),ObstacleIdx(kk,2))]);
%             C_v(kk) = C_v(kk)/dist_ot(kk);
%  end

for i = 1:Ny
    for j = 1:Nx
        Psi_circular = 0;
        Psi_vortex = 0;
        
        for k = 1:No

            dist = norm([X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)),Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))]);
            
            Z = Y(i,j)-Y(TargetIdx(1),TargetIdx(2))+1i*(X(i,j)-X(TargetIdx(1),TargetIdx(2)));
            B = Y(ObstacleIdx(k,1),ObstacleIdx(k,2))-Y(TargetIdx(1),TargetIdx(2))+...
                    1i*(X(ObstacleIdx(k,1),ObstacleIdx(k,2))-X(TargetIdx(1),TargetIdx(2)));
            B_conj = conj(B);
                          
            f = conj(-log(r_o^2/(Z-B)+B_conj));
            
            if dist <= l_o
                Psi_circular =  imag(f); 
                Psi_vortex =  C_v(k)*log(dist^2);
%                 Psi_dynamic =   exp(C_dynamic/dist)*dynamic_flag*...
%                                  imag(-1i*V_y(k)*(r_o^2/(Z-B)+conj(B))-V_x(k)*(r_o^2/(Z-B)+conj(B)));
                break;
            else            
                Psi_circular =  Psi_circular + imag(f);
                Psi_vortex = Psi_vortex + C_v(k)*log(dist^2);
%                 Psi_dynamic =  Psi_dynamic + exp(C_dynamic/dist)*dynamic_flag*...
%                                              imag(1*V_y(k)*(r_o^2/(Z-B)+conj(B))-V_x(k)*(r_o^2/(Z-B)+conj(B)));                
            end
        end        
        
        Psi_s(i,j) =   C_cir*Psi_circular + vortex_flag*Psi_vortex+...
                        abs(atan2(X(i,j)-X(TargetIdx(1),TargetIdx(2)),Y(i,j)-Y(TargetIdx(1),TargetIdx(2))));
                    
%         if dist < r_o
%             Psi_s(i,j) = NaN;
%         end
    end
end

psi_s = reshape(Psi_s,[Ny*Nx,1]);

end


