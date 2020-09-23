%% Matlab simulation of a APF path-planning system
clear all;
close all;

%% Initialization
r2d = 180/pi;
path_length = 0;
heading_change = 0;

% 1: Transit with 1 target; 2: Transit with 2 targets; 3: Transit with 3 target
Maneuvering_mode = 1; 

switch Maneuvering_mode
    case 1
        Lx = 100;
        Ly = 100;
        Nx = 10; dX = Lx/Nx;
        Ny = 10; dY = Ly/Ny;
        N = Nx*Ny;
        idx0 = 10;
    case 2
        Lx = 100;
        Ly = 100;
        Nx = 10; dX = Lx/Nx;
        Ny = 10; dY = Ly/Ny;
        N = Nx*Ny;
        idx0 = 10;
    case 3
        Lx = 200;
        Ly = 200;
        Nx = 20; dX = Lx/Nx;
        Ny = 20; dY = Ly/Ny;
        N = Nx*Ny;
        idx0 = 100;
    otherwise
        Lx = 100;
        Ly = 100;
        Nx = 10; dX = Lx/Nx;
        Ny = 10; dY = Ly/Ny;
        N = Nx*Ny;
        idx0 = 10;
end

d0 = max(dX,dY)+1;

[X,Y] = meshgrid(dX/2:dX:Lx,dY/2:dY:Ly);
x = reshape(X,[N,1]);
y = reshape(Y,[N,1]);
A = zeros(size(X)); % Adjacency matrix

for i=1:N
    for j=1:N
        dist = max(abs(x(j)-x(i)),abs(y(j)-y(i)));
        if (i ~= j) && (dist <= d0)
            A(i,j) = 1;
        end
    end
end

% Vehicle parameters
dT     = 0.01;   % Sample time
L      = 10; % Vehicle length
Delta  = 5; % LOS lookahead distance
Kp     = 0.5; % Proportional gain course controller 
Kappa  = 0.8; % Proportional gain speed controller 
acc_constraint = 0.2; % Constraint on acceleration
yaw_rate_constraint = 0.25; % Constraint on yaw rate
U_max  = 4; % Maxmium speed of vehicle
idx_0   = idx0;
theta0 = 00*pi/180;
r0     = min(dX,dY)/4;

% Potential
U = zeros(size(X));
U_att = zeros(size(X));
U_rep = zeros(size(X));
zeta_1 = 2;
zeta_2 = 10^7;
rou_0 = max(dX,dY)+2;

% Obstacles
switch Maneuvering_mode
    case 1    
        ObstacleIdx = [ 6, 2;
                        6, 3;
                        6, 4;
                        6, 5;
                        6, 6;
                        6, 7;
                        4, 1;
                        4, 2;
                        4, 3;
                        4, 4;
                        4, 5;
                        4, 6;
                        2, 3;
                        2, 4;
                        2, 5;
                        2, 6;
                        2, 7;
                        2, 8;
                        3, 8;
                        4, 8;
                        5, 8;
                        6, 8;
                        7, 8;
                        8, 8;
                        9, 8;
                        10, 8 ];    
    case 2    
        ObstacleIdx = [ 5, 1;
                        5, 2;
                        5, 3;
                        5, 4;
                        5, 6;
                        5, 7;
                        2, 8;
                        3, 8;
                        4, 8;
                        5, 8;
                        6, 8;
                        7, 8;
                        8, 8;
                        9, 8;
                        10, 8];
     case 3
        ObstacleIdx = [ 1,15;
                        2,13;
                        3,13;
                        4,13;
                        5,13;
                        6,13;
                        7,13;
                        8,13;
                        2,14;
                        2,15;
                        8,14;
                        8,15;
                        9,15;
                        10,15;
                        11,15;
                        1,8;
                        2,8;
                        3,8;
                        4,8;
                        5,8;
                        6,7;
                        5,7;
                        6,6;
                        4,8;
                        7,6;
                        8,6;
                        4,1;
                        4,2;
                        4,3;
                        15,8;
                        16,8;
                        17,8;
                        18,8;
                        19,8;
                        20,8;
                        15,17;
                        16,17;
                        17,17;
                        18,17;
                        19,17;
                        15,9;
                        15,10;
                        15,11;
                        15,12
                        ];
    otherwise
        ObstacleIdx = [ ];
       
end
[No,N2] = size(ObstacleIdx);


% Targets
switch Maneuvering_mode
    case 1
        TargetIdx = [ 1, 10 ];  % 1 final target
    case 2     
        TargetIdx = [  2,  1 ;   % 2 specific targets
                      Nx, Ny ];
    case 3
        TargetIdx = [ 1, 1 ;     % 3 specific targets         
                      20,20;
                      5, 15];    
    otherwise
        TargetIdx = [ Nx, Ny ]; % 1 final target
end                    
% [No,~] = size(ObstacleIdx);
[Nt,N2] = size(TargetIdx);

ENV = zeros(Nx,Ny); 
for i=1:Nx
    for j=1:Ny
        for k=1:No
        dist = max(abs(X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2))),...
               abs(Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2))));
        if  (dist <= rou_0)     
            ENV(i,j) = -1;
        end
        end
    end
end
for i=1:No
    ENV(ObstacleIdx(i,1),ObstacleIdx(i,2)) = 1; % Set obstacle as 1 
end
for i=1:Nt
    ENV(TargetIdx(i,1),TargetIdx(i,2)) = i+1; % Set targets as 2,...,Nt+1
end
env = reshape(ENV,[N,1]);

% Initial condition
idx_pre = idx_0;
idx_next = idx_0;

path_idx(1) = idx_0;
path_idx_k = 1;
target_idx = find(env==2);
target_current = 1; % target in sequence
local_min_k = 1; % index for local min

p = [x, y];
path0   = [p(idx_0,:)'; theta0]; % Initial path position
q0      =  p(idx_0,1:2)';       % Initial vehicle position
v0      = [0;0];                % Initial vehicle velocity
q_new   = q0;                   % Next vehicle position
v_surge_old   = v0;
theta0  = rad2pipi(theta0);     % Next vehicle course angle
theta_new = theta0;             % Next vehicle course angle
xi(1,:) = [q0',theta0];         % Guide vehicle state
kk = 2;
theta_p_new     = theta0;
path_old        = path0; 
kk_long(1) = 1;
kk_long_idx = 1;

% Flag
repeat = 0;
Wall_Following_flag = 0;
Wall_Following_enter = 0;

while (1)
    U = zeros(size(X));
    U_att = zeros(size(X));
    U_rep = zeros(size(X));
    % Attractive potential field
    for i=1:Nx
        for j=1:Ny
            p_rt = sqrt((X(i,j)-X(TargetIdx(target_current,1),TargetIdx(target_current,2)))^2+ ...
                (Y(i,j)-Y(TargetIdx(target_current,1),TargetIdx(target_current,2)))^2);
            U_att(i,j) = 0.5*zeta_1*p_rt^2;        
        end
    end
    % Repulsive potential field
    for i=1:Nx
        for j=1:Ny
            rou_val = 100;
            rou = 0;
            if ENV(i,j) == 1  %Obstacle
               U_rep(i,j) = U_rep(i,j)+5*10^3;
            elseif ENV(i,j) < 0
               for k=1:No
                   rou_pre = sqrt((X(i,j)-X(ObstacleIdx(k,1),ObstacleIdx(k,2)))^2+ ...
                              (Y(i,j)-Y(ObstacleIdx(k,1),ObstacleIdx(k,2)))^2);
                   if rou_pre<rou_val
                      rou_val = rou_pre;
                      rou = rou_pre;
                   end
               end
               U_rep(i,j) = U_rep(i,j)+0.5*zeta_2*(1/rou-1/rou_0)^2;
            end
        end
    end
    % Total potential field
    U = U_att + U_rep;
    u = reshape(U,[N,1]);    
       
    % Wall following
    if Wall_Following_flag == 0
        [row,col] = find(A(idx_pre,:));
        u_temp = u(idx_pre);
        for i=1:length(col)
            u_next = u(col(i));
            if u_next < u_temp  
                idx_next = col(i);
                u_temp = u_next;
            end
        end
        path_idx_k = path_idx_k+1;
        path_idx(path_idx_k) = idx_next;
        if  idx_pre == idx_next
            repeat = repeat+1;
            if repeat>=10
                Wall_Following_flag = 1;
            end
        end
%         idx_pre = idx_next;
               
    elseif Wall_Following_flag == 1
        if Wall_Following_enter == 0
            Wall_Following_enter = 1;
            local_min_idx(local_min_k) = idx_next;
            local_min_k = local_min_k+1;
            [row_o,col_o] = find(env<0);
            dist_temp = 10^4;
            for i=1:length(row_o)
                dist = sqrt((x(idx_pre)-x(row_o(i)))^2+(y(idx_pre)-y(row_o(i)))^2);
                if dist<dist_temp
                    idx_next = row_o(i);
                    dist_temp = dist;
                end      
            end
            path_idx_k = path_idx_k+1;
            path_idx(path_idx_k) = idx_next;
            idx_avoid_repeat = idx_pre;
%             idx_pre = idx_next;
            dist_target_pre = sqrt((x(idx_next)-x(target_idx))^2+(y(idx_next)-y(target_idx))^2);
            
        elseif Wall_Following_enter == 1
%             while (1)
                dist_temp = 10^4;
                for i=1:length(row_o) 
                    if idx_pre == row_o(i)
                        continue;
                    end
                    if idx_avoid_repeat ~= row_o(i)
                        dist = sqrt((x(idx_pre)-x(row_o(i)))^2+(y(idx_pre)-y(row_o(i)))^2);
                        if dist<dist_temp
                            idx_next = row_o(i);
                            dist_temp = dist;
                        end
                    end 
                end  
                    
                path_idx_k = path_idx_k+1;
                path_idx(path_idx_k) = idx_next;
                idx_avoid_repeat = idx_pre;
%                 idx_pre = idx_next;
                dist_target_next = sqrt((x(idx_next)-x(target_idx))^2+(y(idx_next)-y(target_idx))^2);
                
                if dist_target_next<=dist_target_pre  % termination condition for wall following
                    repeat = 0;
                    Wall_Following_enter = 0;
                    Wall_Following_flag = 0;
                    dist_target_pre = 0;
                    dist_target_next = 0;
%                     break;
                end
                dist_target_pre = dist_target_next;
            end
%         end
    end

    % New subpath rotation
    theta_p_new = atan2(p(idx_next,2)-p(idx_pre,2),p(idx_next,1)-p(idx_pre,1));
    Rt_new = [cos(theta_p_new) sin(theta_p_new); -sin(theta_p_new) cos(theta_p_new)];
    
    k_in = 1;
    while  k_in<15000
%      while (1) % Simulation loop    
        % Evolving vehicle dynamics
        R0        = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)];
        q_err     = Rt_new*(q0-p(idx_pre,1:2)'); % Pos. error from path
        q_tar_err = Rt_new*(q0-p(idx_next,1:2)'); % Pos. error from target
        v         = -Kappa.*q_tar_err/norm(q_tar_err);
        acc       = (v-v_surge_old)/dT;
        acc_surge = acc(1);
        v_surge   = v(1);
        if acc(1) > acc_constraint
            v_surge = v_surge_old + dT*acc_constraint;            
        end
        if norm(v_surge)<U_max
           U_speed = norm(v_surge);
        else
           U_speed = U_max;
        end      
        theta_los = theta_p_new - atan2(q_err(2),Delta);
        theta_los = rad2pipi(theta_los);
        q_err_dot = Rt_new*R0*[U_speed;0];
        theta_los_dot = -(Delta*q_err_dot(2))/(Delta^2+q_err_dot(2)^2);

        q_new     = q0 + dT*(R0*[U_speed;0]);
        yaw_rate = -Kp*rad2pipi(theta0-theta_los)+theta_los_dot;
        if yaw_rate < yaw_rate_constraint
            theta_new = theta0 + dT*(-Kp*rad2pipi(theta0-theta_los)+theta_los_dot);
        else
            theta_new = theta0 + dT*yaw_rate_constraint;
        end
        theta_new = rad2pipi(theta_new);
        q0        = q_new;
        theta0    = theta_new;
        v_surge_old    = v_surge;
        xi(kk,:) = [q0',theta0];
        kk=kk+1;
        
        if norm(q_new-p(idx_next,1:2)') <= r0
            break;
        end      
%         k_in = k_in+1;
    end
    idx_pre = idx_next;

    kk_long_idx = kk_long_idx+1;
    kk_long(kk_long_idx) = kk-1;
    
    
    if idx_next == target_idx
        if target_current<Nt % choose next target
            target_current = target_current+1;
            target_idx = find(env==target_current+1);
        elseif target_current==Nt % reach all targets
            break;
        end
    end
    
end

% Plot path
path_x = zeros(path_idx_k);
path_y = zeros(path_idx_k);
for i=1:path_idx_k
    path_x(i) = x(path_idx(i));
    path_y(i) = y(path_idx(i));
end
figure(4);plot(path_y,path_x,'b','LineWidth',2);hold on; plot(y,x,'k.');
idx = find(env == 1);
plot(y(idx),x(idx),'r*','Linewidth',2);
plot(y(idx_0),x(idx_0),'b*');
idx = find(env >= 2);
plot(y(idx),x(idx),'bo','Linewidth',2);
idx = local_min_idx;
plot(y(idx),x(idx),'go','Linewidth',2);
axis([0 Ly 0 Lx]);


path_ship_x = zeros(kk_long_idx);
path_ship_y = zeros(kk_long_idx);
for i=1:kk_long_idx
    path_ship_x(i) = xi(kk_long(i),1);
    path_ship_y(i) = xi(kk_long(i),2);
    R0 = [cos(xi(kk_long(i),3)) -sin(xi(kk_long(i),3)); sin(xi(kk_long(i),3)) cos(xi(kk_long(i),3))];
    ship = R0*[6*L/9 5*L/9 3*L/9 -5*L/9 -5*L/9 3*L/9 5*L/9 6*L/9; 0 1*L/9 2*L/9 2*L/9 -2*L/9 -2*L/9 -1*L/9 0];
    plot(xi(kk_long(i),2)+ship(2,:), xi(kk_long(i),1)+ship(1,:),'k');
end
for i=1:kk_long_idx-1
    path_length = path_length + norm(xi(kk_long(i),1:2)-xi(kk_long(i+1),1:2));
    heading_change = heading_change + abs(xi(kk_long(i),3)-xi(kk_long(i+1),3))*r2d;
end
% plot(path_ship_y,path_ship_x,'k');