% function [waypoint_n] = Nav_func_NextWaypoint(Psi_p,delta_Psi_p,ObstacleIdx,TargetIdx,V_obs,waypoint_c,X,Y,Nx,Ny,n)
% % NextWaypoint finds the next waypoint from adjacent points by choosing the one with lowest potential value 
% % 
% % Hongyu Zhou
% % 
% 
% max_potential = 100;
% waypoint_n = [];
% idx_c_y = waypoint_c(1);
% idx_c_x = waypoint_c(2);
% 
% [No,~] = size(ObstacleIdx);
% dist = zeros(1,No);
% dist_tar = 0;
% for k=1:No
%     dist(k) = norm(waypoint_c-ObstacleIdx(k,:));
% end
% dist_min = min(dist);
% 
% headon_flag = 1;
% delta = delta_Psi_p(idx_c_y,idx_c_x);
% gamma = 0;
% alpha = 1;
% 
% 
% % % Decide head on situation
% % lateral_dist = 0;
% % long_dist = 0;
% % V_obs_y = V_obs(:,1);
% % V_obs_x = V_obs(:,2);
% % idx = find(V_obs_x~=0);
% % [N_idx,~] = size(idx);
% % for k = 1:N_idx
% %     lateral_dist = ObstacleIdx(k,1)-waypoint_c(1); 
% %     long_dist = ObstacleIdx(k,2)-waypoint_c(2);
% %     long_relative_pos  = (ObstacleIdx(k,2)-waypoint_c(2))*(ObstacleIdx(k,2)-TargetIdx(2)); % if long_pos>0 then the robot already past the obstacle
% %     if (V_obs_y(k)==0) && (long_relative_pos<0) && (abs(lateral_dist)<=n) && (abs(long_dist)<=5*n)
% %         headon_flag = 0;
% %         break;
% %     end
% % end
% 
%  for i=(idx_c_y-n):(idx_c_y+headon_flag*n)
%      if i<1 || i>Ny 
%         continue;
%      end
%      
%      for j=(idx_c_x-n):(idx_c_x+n)
%         if j<1 || j>Nx 
%             continue;
%         end
%         
%         if i == idx_c_y && j == idx_c_x
%             continue;
%         end
%         
% %         dist_tar = norm([X(i,j)-X(TargetIdx(1),TargetIdx(2)),Y(i,j)-Y(TargetIdx(1),TargetIdx(2))]);
% 
% %         if dist_min<=10
% %             gamma = 0.05; % Mode 1-9
% %             if delta < delta_Psi_p(i,j)
% %                 continue;
% %             else
%                 val_potential = Psi_p(i,j);
%                 if val_potential < max_potential  
%                     waypoint_n = [i,j];
%                     max_potential = val_potential;
%                 end
%             end
%             
%         
%      end
%  end





function [waypoint_n] = Nav_func_NextWaypoint(Psi_p,delta_Psi_p,ObstacleIdx,TargetIdx,V_obs,waypoint_c,X,Y,Nx,Ny,n)
% NextWaypoint finds the next waypoint from adjacent points by choosing the one with lowest potential value 
% 
% Hongyu Zhou
% 

max_potential = 100;
waypoint_n = [];
idx_c_y = waypoint_c(1);
idx_c_x = waypoint_c(2);

[No,~] = size(ObstacleIdx);
dist = zeros(1,No);
dist_tar = 0;
for k=1:No
    dist(k) = norm(waypoint_c-ObstacleIdx(k,:));
end
dist_min = min(dist);

headon_flag = 1;
delta = delta_Psi_p(idx_c_y,idx_c_x);
gamma = 0;
alpha = 1;


% Decide head on situation
lateral_dist = 0;
long_dist = 0;
V_obs_y = V_obs(:,1);
V_obs_x = V_obs(:,2);
idx = find(V_obs_x~=0);
[N_idx,~] = size(idx);
for k = 1:N_idx
    lateral_dist = ObstacleIdx(k,1)-waypoint_c(1); 
    long_dist = ObstacleIdx(k,2)-waypoint_c(2);
    long_relative_pos  = (ObstacleIdx(k,2)-waypoint_c(2))*(ObstacleIdx(k,2)-TargetIdx(2)); % if long_pos>0 then the robot already past the obstacle
    if (V_obs_y(k)==0) && (long_relative_pos<0) && (abs(lateral_dist)<=n) && (abs(long_dist)<=5*n)
        headon_flag = 0;
        break;
    end
end

 for i=(idx_c_y-n):(idx_c_y+headon_flag*n)
     if i<1 || i>Ny 
        continue;
     end
     
     for j=(idx_c_x-n):(idx_c_x+n)
        if j<1 || j>Nx 
            continue;
        end
        
        if i == idx_c_y && j == idx_c_x
            continue;
        end
        
        dist_tar = norm([X(i,j)-X(TargetIdx(1),TargetIdx(2)),Y(i,j)-Y(TargetIdx(1),TargetIdx(2))]);

        if dist_min<=10
            gamma = 0.05; % Mode 1-9
            if delta < delta_Psi_p(i,j)
                continue;
            else
                val_potential = Psi_p(i,j)+gamma*dist_tar;
                if val_potential < max_potential  
                    waypoint_n = [i,j];
                    max_potential = val_potential;
                end
            end
            
        else 
            gamma = 0.05; % Mode 1-9
            val_potential = Psi_p(i,j)+gamma*dist_tar;
                if val_potential < max_potential  
                    waypoint_n = [i,j];
                    max_potential = val_potential;
                end
        end
     end
 end

end
