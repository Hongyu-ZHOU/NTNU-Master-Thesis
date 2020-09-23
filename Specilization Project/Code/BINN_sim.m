%% Matlab simulation of a BINN path-planning system with targets as min
clear all;

%% Configuration

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

% Simulation parameters
dT         = 0.01;   % Sample time
T_init     = 20;   % Long simulation time
N_init     = ceil(T_init/dT);
T_inner    = 150.0;   % Short simulation time
N_inner    = ceil(T_inner/dT);
N_outer    = 91;

% Vehicle parameters
L      = 10; % Vehicle length
Delta  = 5; % LOS lookahead distance
Kp     = 0.5; % Proportional gain course controller 
Kappa  = 0.8; % Proportional gain speed controller 
acc_constraint = 0.2; % Constraint on acceleration
yaw_rate_constraint = 0.25; % Constraint on yaw rate
U_max  = 4; % Maxmium speed of vehicle
theta0 = 00*pi/180;

% NN parameters
Gamma  = 100;
lambda = 0.5;  % heading penalty
d0     = max(dX,dY)+1;
r0     = min(dX,dY)/4;
a      = 50;
bu     = 1.0;
bl     = 1.0;
mu     = 20.0;

A = zeros(N,N); % Adjacency matrix
W = zeros(N,N); % Weight matrix

[X,Y] = meshgrid(dX/2:dX:Lx,dY/2:dY:Ly);
Z = zeros(size(X));
x = reshape(X,[N,1]);
y = reshape(Y,[N,1]);

z_old = zeros(size(x));
z_new = z_old;

for i=1:N
    for j=1:N
        dist1 = max(abs(x(j)-x(i)),abs(y(j)-y(i)));
        dist2 = sqrt((x(j)-x(i))^2+(y(j)-y(i))^2);
        if (i ~= j) && (dist1 <= d0)
            A(i,j) = 1;
            W(i,j) = mu/dist2;
        end
    end
end


%% Targets and obstacles
UPS   = zeros(size(X));

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
for i=1:No
%     UPS(ObstacleIdx(i,1),ObstacleIdx(i,2)) = -Gamma;
    UPS(ObstacleIdx(i,1),ObstacleIdx(i,2)) = Gamma;
end

% Targets
switch Maneuvering_mode
    case 1
        TargetIdx = [ 1, 10 ];  % 1 final target
    case 2     
        TargetIdx = [  2,  1 ;   % 2 specific targets
                      Nx, Ny ];
    case 3
        TargetIdx = [ 5, 15;     % 3 specific targets
                      1, 1 ;
                      20,20];    
    otherwise
        TargetIdx = [ Nx, Ny ]; % 1 final target
end
[Nt,N2] = size(TargetIdx);
for i=1:Nt
%     UPS(TargetIdx(i,1),TargetIdx(i,2)) = Gamma;
    UPS(TargetIdx(i,1),TargetIdx(i,2)) = -Gamma;
end
ups = reshape(UPS,[N,1]);

% figure(1); clf; hold off;
% surf(X,Y,Z); xlabel('x'); ylabel('y');


%% Simulating the neural network initial landscape

for k = 1:N_init  % Simulation loop
    for i=1:N
        ups_p = max(0,ups(i));
        ups_m = max(0,-ups(i));
        excite = 0.0;
        [row,col] = find(A(i,:));
        for jj=1:length(col)
%                 z_p    = max(0,z_old(col(jj)));
                z_m    = max(0,-z_old(col(jj)));
%                 excite = excite + W(i,col(jj))*z_p;
                excite = excite + W(i,col(jj))*z_m;
            end
%             z_new(i) = z_old(i) + dT*(-a*z_old(i)-(z_old(i)-bu)*(ups_p+excite)-(z_old(i)+bl)*ups_m);
            z_new(i) = z_old(i) + dT*(-a*z_old(i)-(z_old(i)+bl)*(ups_m+excite)-(z_old(i)-bu)*ups_p);
    end
    z_old = z_new;
end
p = [x, y, z_new];

% Z  = reshape(z_old,[Nx,Ny]);
% surf(Y,X,Z); xlabel('y'); ylabel('x');
% pause;
    
%% Simulating the motion of a guide vehicle

path0   = [p(idx0,:)'; theta0]; % Initial path position
q0      = [p(idx0,1:2)'];       % Initial vehicle position
v0      = [0;0];                % Initial vehicle velocity
q_new   = q0;                   % Next vehicle position
v_surge_old   = v0;
theta0  = rad2pipi(theta0);     % Next vehicle course angle
theta_new = theta0;             % Next vehicle course angle
xi      = zeros(N_outer*N_inner+1,3);
xi(1,:) = [q0',theta0];         % Guide vehicle state

idx_old = idx0; 
idx_new = idx0;

theta_p_new     = theta0;
path_old        = path0; 
path_new_idx    = zeros(N_outer+1,1);
path_new_idx(1) = idx0;
ups(idx_new)    = 0.0;
z_new(idx_new)  = 0.0;
z_old(idx_new)  = 0.0;

p = [x, y, z_new];

kk_long = zeros(N_outer+1,1);
kk = 2; kk_long(1) = 1;
for j = 1:N_outer  % Outer simulation loop
    [row,col] = find(A(idx_old,:));
    max_val = 0.0;
    for jj=1:length(col)
        theta_p     = atan2(p(col(jj),2)-p(idx_old,2),p(col(jj),1)-p(idx_old,1));
        theta_p_err = abs(rad2pipi(theta_p-path_old(4)));
        val = (1-lambda/pi*theta_p_err)*p(col(jj),3);
        if val < max_val  % Targets minima
%         if val > max_val  % Targets maxima
            idx_new = col(jj);
            max_val = val;
            theta_p_new = theta_p;
        end
    end
    % New subpath rotation
    Rt_new = [cos(theta_p_new) sin(theta_p_new); -sin(theta_p_new) cos(theta_p_new)];
    
    % Update vectors
    ups(idx_new)    = 0.0;
    z_old(idx_new)  = 0.0;
    z_new(idx_new)  = 0.0;
    path_new_idx(j+1) = idx_new;
    
    k = 1;
    while k <= N_inner  % Simulation loop
        % Evolving NN dynamics
        for i=1:N
            ups_p = max(0,ups(i));
            ups_m = max(0,-ups(i));
            excite = 0.0;
            [row,col] = find(A(i,:));
            for jj=1:length(col)
%                 z_p    = max(0,z_old(col(jj)));
                z_m    = max(0,-z_old(col(jj)));
%                 excite = excite + W(i,col(jj))*z_p;
                excite = excite + W(i,col(jj))*z_m;
            end
%             z_new(i) = z_old(i) + dT*(-a*z_old(i)-(z_old(i)-bu)*(ups_p+excite)-(z_old(i)+bl)*ups_m);
            z_new(i) = z_old(i) + dT*(-a*z_old(i)-(z_old(i)+bl)*(ups_m+excite)-(z_old(i)-bu)*ups_p);
        end
        z_old = z_new;
        
        % Evolving vehicle dynamics
        R0        = [cos(theta0) -sin(theta0); sin(theta0) cos(theta0)];
        q_err     = Rt_new*(q0-p(idx_old,1:2)'); % Pos. error from path
        q_tar_err = Rt_new*(q0-p(idx_new,1:2)'); % Pos. error from target
        v         = -Kappa.*q_tar_err/norm(q_tar_err);
        acc       = (v-v_surge_old)/dT;
        acc_surge = acc(1);
        v_surge   = v(1);
        if acc(1) > acc_constraint
            v_surge = v_surge_old + dT*acc_constraint;            
        end
        if norm(v_surge)<U_max
           U = norm(v_surge);
        else
           U = U_max;
        end      
        theta_los = theta_p_new - atan2(q_err(2),Delta);
        theta_los = rad2pipi(theta_los);
        q_err_dot = Rt_new*R0*[U;0];
        theta_los_dot = -(Delta*q_err_dot(2))/(Delta^2+q_err_dot(2)^2);

        q_new     = q0 + dT*(R0*[U;0]);
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
        
        if norm(q_new-p(idx_new,1:2)') <= r0
            z_old(idx_new)  = 0.0;
            z_new(idx_new)  = 0.0;
            break;
        end
        k=k+1;
    end    
    p         = [x, y, z_new];
    path_old  = [p(idx_new,:)'; theta_p_new];
    idx_old   = idx_new;
    kk_long(j+1) = kk-1;
    

    Z  = reshape(z_new,[Nx,Ny]);
%     surf(Y,X,Z); xlabel('y'); ylabel('x');

    if j == N_outer
        j;
    end
    % Plotting the neurons
    clf;
    idx = find(z_new>eps & z_new<0.5);
    plot(y(idx),x(idx),'k.','Linewidth',2); 
    hold on;
    idx = find(z_new>=0.5);
    plot(y(idx),x(idx),'r+','Linewidth',2); 
    idx = find(z_new<-eps & z_new>-0.5);
    plot(y(idx),x(idx),'k.','Linewidth',2); 
    idx = find(z_new<=-0.5);
    plot(y(idx),x(idx),'bo','Linewidth',2); 
    idx = find(abs(z_new)<=eps);
    plot(y(idx),x(idx),'.k','Linewidth',2); xlabel('y'); ylabel('x');
    
    % Plotting the path and vehicle tracks
    idx = find(path_new_idx>0.5);
    plot(p(path_new_idx(idx),2),p(path_new_idx(idx),1),'m','LineWidth',1);
%     plot3(q(xi_new_idx(idx),2),q(xi_new_idx(idx),1),q(xi_new_idx(idx),3),'m','LineWidth',2);
    plot(xi(1:kk-1,2),xi(1:kk-1,1),'b','LineWidth',2);
    axis([0 Ly 0 Lx]);
    
    for i = 1:j+1
        R0 = [cos(xi(kk_long(i),3)) -sin(xi(kk_long(i),3)); sin(xi(kk_long(i),3)) cos(xi(kk_long(i),3))];
        ship = R0*[6*L/9 5*L/9 3*L/9 -5*L/9 -5*L/9 3*L/9 5*L/9 6*L/9; 0 1*L/9 2*L/9 2*L/9 -2*L/9 -2*L/9 -1*L/9 0];
        plot(xi(kk_long(i),2)+ship(2,:), xi(kk_long(i),1)+ship(1,:),'k');
%             patch(xi(kk-1,2)+ship(2,:), xi(kk-1,1)+ship(1,:),[0,0,0]);
    end

    hold off;
    pause(0.05);
    switch Maneuvering_mode
        case 1
            if norm(xi(kk-1,1:2)-[95;5]') <= r0
            break;  
            end
        case 2
            if norm(xi(kk-1,1:2)-[95;95]') <= r0
            break;             
            end
        case 3
            if norm(xi(kk-1,1:2)-[145;45]') <= r0
            break;             
            end                  
    end
end

kk_long_length = length(find(kk_long~=0))-1;
for i=1:kk_long_length
    path_length = path_length + norm(xi(kk_long(i),1:2)-xi(kk_long(i+1),1:2));
    heading_change = heading_change + abs(xi(kk_long(i),3)-xi(kk_long(i+1),3))*r2d;
end




