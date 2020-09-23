function [waypoint_n] = Stream_func_NextWaypoint(Psi_s,waypoint_c,ObstacleIdx,TargetIdx,l_o,X,Y,Nx,Ny,n)
% NextWaypoint finds the next waypoint from adjacent points by choosing the one with lowest potential value 
% 
% Hongyu Zhou
% 


max_delta= inf;
min_dist = 0;
waypoint_n = [];
idx_c_y = waypoint_c(1);
idx_c_x = waypoint_c(2);
Psi_current = Psi_s(idx_c_y,idx_c_x);

[No,~] = size(ObstacleIdx);
dist = zeros(1,No);
gamma = 0.2;

% dist_c2t = norm([X(idx_c_y,idx_c_x)-X(TargetIdx(1),TargetIdx(2)),Y(idx_c_y,idx_c_x)-Y(TargetIdx(1),TargetIdx(2))]);
% if dist_c2t<=sqrt(2) % n*dX*sqrt(2)
%    waypoint_n = TargetIdx;   
if abs(X(idx_c_y,idx_c_x)-X(TargetIdx(1),TargetIdx(2)))<=1 && ... % n*0.2=1
        abs(Y(idx_c_y,idx_c_x)-Y(TargetIdx(1),TargetIdx(2)))<=1
    waypoint_n = TargetIdx;
else
     for i=(idx_c_y-n):(idx_c_y+n)  
         for j=(idx_c_x-n):(idx_c_x+n)
             if max(abs([i j]-waypoint_c))<n
                 continue;
             end
         
            dist = zeros(1,No);
            % skip the grid points inside obstacles
            for k=1:No
                dist(k) = norm([X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)),Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))]);
            end
            min_dist = min(dist);
            if min_dist <= l_o
                continue;
            end
            
            dist_tar = norm([X(i,j)-X(TargetIdx(1),TargetIdx(2)),Y(i,j)-Y(TargetIdx(1),TargetIdx(2))]);

            % find the waypoint
                val_stream = abs(Psi_s(i,j)-Psi_current)+gamma*dist_tar;
                if val_stream < max_delta  
                    waypoint_n = [i,j];
                    max_delta = val_stream;
                end
         end
     end
end

end




% function [waypoint_n] = Stream_func_NextWaypoint(Psi_s,waypoint_c,ObstacleIdx,TargetIdx,l_o,X,Y,Nx,Ny,n)
% % NextWaypoint finds the next waypoint from adjacent points by choosing the one with lowest potential value 
% % 
% % Hongyu Zhou
% % 
% 
% 
% max_delta= 1;
% min_dist = 0;
% waypoint_n = [];
% idx_c_y = waypoint_c(1);
% idx_c_x = waypoint_c(2);
% Psi_current = Psi_s(idx_c_y,idx_c_x);
% 
% [No,~] = size(ObstacleIdx);
% dist = zeros(1,No);
% 
% 
% 
% j=(idx_c_x-n);
% 
% if abs(TargetIdx(2)-idx_c_x)<=n
%      waypoint_n = TargetIdx;   
% else
%      for i=(idx_c_y-n):(idx_c_y+n)  
%         dist = zeros(1,No);
%         % skip the grid points inside obstacles
%         for k=1:No
%             dist(k) = norm([X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)),Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))]);
%         end
%         min_dist = min(dist);
%         if min_dist <= l_o
%             continue;
%         end
% 
%         % find the waypoint
%             val_stream = abs(Psi_s(i,j)-Psi_current);
%             if val_stream < max_delta  
%                 waypoint_n = [i,j];
%                 max_delta = val_stream;
%             end
%      end
% end
% 
% end












