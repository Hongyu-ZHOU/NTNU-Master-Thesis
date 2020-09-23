close all;
clear all;
addpath(genpath(pwd));

%% Thruster Allocation
% Thruster Position
Lx1=-0.4574;
Lx2=-0.4574;
Lx3=0.3875;
Ly1=-0.055;
Ly2=0.055;
Ly3=0;

% Matrix: tau to u
B_tau2u = [ 1 0 1 0 0;
            0 1 0 1 1;
           -Ly1 Lx1 -Ly2 Lx2 Lx3];
K_tau2u = diag( [1.03, 1.03, 1.03, 1.03, 2.629]);
C_tau2u = pinv(B_tau2u*K_tau2u);

%% Ship Paramrters
% M-matrix:
m    = 14.11;
I_z  =  1.7600;  
x_g = 0.0375;  % x_g  =  0.0460; % Table B.1

% Added mass
X_ud  =   -2.0;									
Y_vd =    -10.0;	
N_vd =    -0.0;		
Y_rd =    -0.0;
N_rd =    -1.0;		
                    
% Total mass matrix
m_11 = m-X_ud;
m_22 = m-Y_vd;
m_23 = m*x_g-Y_rd;
m_32 = m*x_g-N_vd;
m_33 = I_z-N_rd;

M    = [m_11 0    0
        0    m_22 m_23
        0    m_32 m_33];
    
inv_M = inv(M);
	
% Damping coefficients
X_u	= -0.6555;		
X_v	= 0;
Y_v	= -1.33;
Y_r	= -7.250; 
N_v	= 0;
N_r	= -1.900; 
					
% Assembly of the damping matrix
d_11 = -X_u ;
d_22 = -Y_v;
d_33 = -N_r;
 
d_12 = -X_v;
d_23 = -Y_r;
d_32 = -N_v;
 
 D    = [d_11 0 0
         0 d_22 d_23
         0 d_32 d_33];
     
%% Observor    
% % Defining L's
% L_1 = diag([12 12 12]);
% L_2 = diag([8 8 8]);
% L_3 = diag([1 1 1]);

%% Maneuvering
Umax = 0.05; 
mu = 10e-4; % for tracking update law: mu = 0;
            % for unit-tangent gradient update law,tune

% Tuning Parameters
K_1 = 10*diag([2 2 4]);
K_2 = 10*diag([2 2 2]);
b = [0;0;0]; % constant bias


%% Workspace
% Discretization
Lx = 20; 
Ly = 20;
Nx = 100; dX = Lx/Nx;
Ny = 100; dY = Ly/Ny;
N = Nx*Ny;

[X,Y] = meshgrid(dX/2:dX:Lx-dX/2,dX/2:dY:Ly-dX/2);
x = reshape(X,[N,1]);
y = reshape(Y,[N,1]);

% Radius
r_o = 0.5/1.5;
l_o = 1.5*r_o;
% l_o = 1;
r_world = Lx/2;

% Define obstacles and target
Mode = 1;  % 1-9
[ObstacleIdx,~,C_v] =  ObstacleIdx(Mode);
ObstacleIdx_init = ObstacleIdx;
[No,~] = size(ObstacleIdx);
TargetIdx = [  
                50,20
                        ];


%% Simulation
C_cir = 1;
vortex_flag = 1;  % 0: no vortex component; 1: with vortex component                        
pos_init = [50,80];

waypoint_c = pos_init;

k = 1; % waypoint index
path_k(k,:) = [waypoint_c]; % waypoints
pos(k,:) = [X(path_k(k,1),path_k(k,2)),Y(path_k(k,1),path_k(k,2))]; % waypoint positions
pd_initial = [pos(1,:),pi];
ship_pos = []; % ship position
n = 5; % search range of waypoints
CP_prev = zeros(8,2); % control points
Q = 0; % Path length

while(1)
    [psi_s,Psi_s] = Stream_func_Streamline_vortex(X,Y,Nx,Ny,C_cir,C_v,ObstacleIdx,TargetIdx,l_o,r_o,vortex_flag); % Generate the streamlines

    [waypoint_n] = Stream_func_NextWaypoint(Psi_s,waypoint_c,ObstacleIdx,TargetIdx,l_o,X,Y,Nx,Ny,n); % Next waypoint
    
    k = k+1;
    path_k(k,:) = [waypoint_n];
    pos(k,:) = [X(path_k(k,1),path_k(k,2)),Y(path_k(k,1),path_k(k,2))];
    pd_final = [X(path_k(k,1),path_k(k,2));Y(path_k(k,1),path_k(k,2))]; % the final position
     % % Pagmatic approach   
%     [CP,Bezier] = Path_generation(pos,k,CP_prev, pd_initial);
%     Bezier_con = [Bezier.B_matrix Bezier.dot_B_matrix Bezier.ddot_B_matrix Bezier.dddot_B_matrix];

    % Quadratic programming  
    [CP,Bezier_con,Q] = Path_generation_qp(pos,k,CP_prev, pd_initial,Q);
    
    
    sim('CSE1_Sim.slx');    
    pd_initial = [eta_sim.signals.values(end,:)]; % initial position 
    ship_pos = [ship_pos ; eta_sim.signals.values];
    
    % PLot
    centers = zeros(No,2);
    radius = zeros(No,1);
    figure(1);clf; hold off;
%     plot(pos(1,2),pos(1,1),'ro','LineWidth',2);hold on;
    plot(Y(TargetIdx(1),TargetIdx(2)),X(TargetIdx(1),TargetIdx(2)),'r*');hold on;
    for i=1:No
        centers(i,:) = [Y(ObstacleIdx(i,1),ObstacleIdx(i,2)),X(ObstacleIdx(i,1),ObstacleIdx(i,2))];
        radius(i) = r_o;
        plot(Y(ObstacleIdx(i,1),ObstacleIdx(i,2)),X(ObstacleIdx(i,1),ObstacleIdx(i,2)),'r+','Linewidth',2);hold on;
        plot([Y(ObstacleIdx(i,1),ObstacleIdx(i,2)),Y(ObstacleIdx_init(i,1),ObstacleIdx_init(i,2))],...
              [X(ObstacleIdx(i,1),ObstacleIdx(i,2)),X(ObstacleIdx_init(i,1),ObstacleIdx_init(i,2))],...
              '-.','Linewidth',2);hold on;    
    end
    viscircles(centers,radius,'Color','k');hold on;
    contour(Y,X,Psi_s,'LevelStep',0.1);
%     contour(Y,X,Psi_s,'LevelStep',0.1,'ShowText','on','LabelSpacing',1500);
    plot(Y(TargetIdx(1),TargetIdx(2)),X(TargetIdx(1),TargetIdx(2)),'r*');hold on;
    plot(pos(:,2),pos(:,1),'ko','LineWidth',2);hold on;
    plot(ship_pos(:,2),ship_pos(:,1),'k');hold on;
    xlabel('y(m)'); ylabel('x(m)');
    legend('Target','Obstacle');
%     pause;
    
    if waypoint_n == TargetIdx
        break;
    end  
    
    waypoint_c = waypoint_n; 
    CP_prev = CP;
    [ObstacleIdx] = Stream_func_MovingObstavcle(ObstacleIdx,X,Y,r_world,l_o,k,n,Mode);
    % pause;
end



