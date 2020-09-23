function [ObstacleIdx,V_obs,C_v] =  ObstacleIdx(Mode)
% ObstacleIdx defines the index of obstacles
% 
% Hongyu Zhou
% 
switch Mode
    
    case 1 % One obstacle, Head on
        ObstacleIdx = [                
                         50,60;
                                ]; 
        V_obs = [
                    0,-1
                            ];
        C_v = [0.1];
        
    case 2 % One obstacle, Head on
        ObstacleIdx = [                
                         50,60;
                                ];                             
        V_obs = [
                    0,1
                            ];
        C_v = [0.1];
        
    case 3 % One obstacle, Crossing
        ObstacleIdx = [                
                         55,55;
                                ];   
        V_obs = [
                    -1,0;

                            ]; 
        C_v = [-0.1];
        
    case 4 % One obstacle, Crossing
        ObstacleIdx = [                
                         45,55;
                                ]; 
        V_obs = [
                    1,0
                            ];     
        C_v = [0.1];
        
    case 5 % Three obstacles, head on
        ObstacleIdx = [                
                         50,60;
                         40,40;
                         60,30
                                ]; 
        V_obs = [
                    0,1;
                    0,1;
                    0,1
                            ]; 
        C_v = [0.05;0.2;0.1];
        
    case 6 % Three obstacles, Crossing
        ObstacleIdx = [                
                         50,60;
                         40,50;
                         30,40;
                                ];   
        V_obs = [
                    1,0;
                    1,0;
                    1,0
                            ]; 
        C_v = [0.1;0.1;0.1];     
        
    case 7 % Three obstacles, Crossing
        ObstacleIdx = [                
                         50,65;
                         40,50;
                         80,30;
                                ];   
        V_obs = [
                    -1,0;
                    1,0;
                    -1,0
                            ]; 
        C_v = [-0.1;0.15;-0.2];

    case 8  % Five obstacles, complex
        ObstacleIdx = [                
                         50,70;
                         50,55;
                         70,50;
                         20,21;
                         40,21
                                ];   
        V_obs = [
                    0,1;
                    -1,0.4;
                    -1,0.2;
                    1,0.2;
                    1,0.4
                            ]; 
        C_v = [0.1;-0.1;-0.1;0.15;0.2];
    case 9 % Five obstacles, complex
        ObstacleIdx = [                
                         50,65;
                         60,45;
                         60,40;
                         80,25;
                         40,21
                                ];   
        V_obs = [
                    1,1;
                    -1.4,0.4;
                    -1.4,-0.2;
                    -1,0;
                    1,1
                            ]; 
        C_v = [0.1;-0.1;-0.1;-0.1;0.2];

end

end

