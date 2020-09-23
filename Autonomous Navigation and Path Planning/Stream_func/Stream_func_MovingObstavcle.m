function  [ObstacleIdx] = Stream_func_MovingObstavcle(ObstacleIdx,X,Y,r_world,r_o,k,n,Mode)
% Stream_func_MovingObstavcle calculates the positions of moving obstacles
% 
% Hongyu Zhou
% 

switch Mode
    
    case 1 % One obstacle, Overtaking
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];

        if k >= 1 && k <= 4
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(2) = Obstacle_1(2)-n;   
            end
        end
        ObstacleIdx(1,:) = Obstacle_1;
        
    case 2 % One obstacle, Head on
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];

        if k >= 1 && k <= 4
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(2) = Obstacle_1(2)+n;   
            end
        end
        ObstacleIdx(1,:) = Obstacle_1;

    case 3 % One obstacle, Crossing
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];

        if k >= 1 && k <= 8
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(1) = Obstacle_1(1)-n;
            end          
        end
        ObstacleIdx(1,:) = Obstacle_1;
        
    case 4 % One obstacle, Crossing
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];

        if k >= 1 && k <= 8
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(1) = Obstacle_1(1)+n;
            end          
        end
        ObstacleIdx(1,:) = Obstacle_1;
        
    case 5 % Three obstacles, head on
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        Obstacle_2 = ObstacleIdx(2,:); % Obstacle #2
        Obstacle_3 = ObstacleIdx(3,:); % Obstacle #3

        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];
        pos_2 = [Y(Obstacle_2(1),Obstacle_2(2)),X(Obstacle_2(1),Obstacle_2(2))];
        pos_3 = [Y(Obstacle_3(1),Obstacle_3(2)),X(Obstacle_3(1),Obstacle_3(2))];

        if k >= 1 && k <= 7
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(2) = Obstacle_1(2)+n;   
            end
        end

        if k >= 1 && k <= 9
            if norm(pos_2-r_world)+r_o <= r_world
                Obstacle_2(2) = Obstacle_2(2)+n;
            end
        end

        if k >= 1 && k <= 9
            if norm(pos_3-r_world)+r_o <= r_world
                Obstacle_3(2) = Obstacle_3(2)+n;
            end
        end

        ObstacleIdx(1,:) = Obstacle_1;
        ObstacleIdx(2,:) = Obstacle_2;
        ObstacleIdx(3,:) = Obstacle_3;
        
    case 6 % Three obstacles, Crossing
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        Obstacle_2 = ObstacleIdx(2,:); % Obstacle #2
        Obstacle_3 = ObstacleIdx(3,:); % Obstacle #3

        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];
        pos_2 = [Y(Obstacle_2(1),Obstacle_2(2)),X(Obstacle_2(1),Obstacle_2(2))];
        pos_3 = [Y(Obstacle_3(1),Obstacle_3(2)),X(Obstacle_3(1),Obstacle_3(2))];

        if k >= 1 && k <= 9
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(1) = Obstacle_1(1)+n;
            end       
        end

        if k >= 1 && k <= 10
            if norm(pos_2-r_world)+r_o <= r_world
                Obstacle_2(1) = Obstacle_2(1)+n;
            end
        end

        if k >= 5 && k <= 12
            if norm(pos_3-r_world)+r_o <= r_world
                Obstacle_3(1) = Obstacle_3(1)+n;
            end
        end

        ObstacleIdx(1,:) = Obstacle_1;
        ObstacleIdx(2,:) = Obstacle_2;
        ObstacleIdx(3,:) = Obstacle_3;
        
    case 7 % Three obstacles, Crossing, complex
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        Obstacle_2 = ObstacleIdx(2,:); % Obstacle #2
        Obstacle_3 = ObstacleIdx(3,:); % Obstacle #3

        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];
        pos_2 = [Y(Obstacle_2(1),Obstacle_2(2)),X(Obstacle_2(1),Obstacle_2(2))];
        pos_3 = [Y(Obstacle_3(1),Obstacle_3(2)),X(Obstacle_3(1),Obstacle_3(2))];

        if k >= 1 && k <= 8
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(1) = Obstacle_1(1)-n;
            end       
        end

        if k >= 3 && k <= 8
            if norm(pos_2-r_world)+r_o <= r_world
                Obstacle_2(1) = Obstacle_2(1)+n;
            end
        end

        if k >= 1 && k <= 16
            if norm(pos_3-r_world)+r_o <= r_world
                Obstacle_3(1) = Obstacle_3(1)-n;
            end
        end

        ObstacleIdx(1,:) = Obstacle_1;
        ObstacleIdx(2,:) = Obstacle_2;
        ObstacleIdx(3,:) = Obstacle_3;
    case 8 % Five obstacles, complex
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        Obstacle_2 = ObstacleIdx(2,:); % Obstacle #2
        Obstacle_3 = ObstacleIdx(3,:); % Obstacle #3
        Obstacle_4 = ObstacleIdx(4,:); % Obstacle #4
        Obstacle_5 = ObstacleIdx(5,:); % Obstacle #5

        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];
        pos_2 = [Y(Obstacle_2(1),Obstacle_2(2)),X(Obstacle_2(1),Obstacle_2(2))];
        pos_3 = [Y(Obstacle_3(1),Obstacle_3(2)),X(Obstacle_3(1),Obstacle_3(2))];
        pos_4 = [Y(Obstacle_4(1),Obstacle_4(2)),X(Obstacle_4(1),Obstacle_4(2))];
        pos_5 = [Y(Obstacle_5(1),Obstacle_5(2)),X(Obstacle_5(1),Obstacle_5(2))];

        if k >= 1 && k <= 4
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(2) = Obstacle_1(2)+n;
            end       
        end

        if k >= 1 && k <= 5
            if norm(pos_2-r_world)+r_o <= r_world
                Obstacle_2(1) = Obstacle_2(1)-n;
                Obstacle_2(2) = Obstacle_2(2)+0.4*n;
            end
        end

        if k >= 1 && k <= 9
            if norm(pos_3-r_world)+r_o <= r_world
                Obstacle_3(1) = Obstacle_3(1)-n;
                Obstacle_2(2) = Obstacle_2(2)+0.6*n;
            end
        end
        
        if k >= 3 && k <= 12
            if norm(pos_4-r_world)+r_o <= r_world
                Obstacle_4(1) = Obstacle_4(1)+n;
                Obstacle_4(2) = Obstacle_4(2)+0.2*n;
            end
        end
        
        if k >= 3 && k <= 12
            if norm(pos_5-r_world)+r_o <= r_world
                Obstacle_5(1) = Obstacle_5(1)+n;
                Obstacle_5(2) = Obstacle_5(2)+0.6*n;
            end
        end

        ObstacleIdx(1,:) = Obstacle_1;
        ObstacleIdx(2,:) = Obstacle_2;
        ObstacleIdx(3,:) = Obstacle_3;
        ObstacleIdx(4,:) = Obstacle_4;
        ObstacleIdx(5,:) = Obstacle_5;
                        
   case 9 % Five obstacles, complex
        Obstacle_1 = ObstacleIdx(1,:); % Obstacle #1
        Obstacle_2 = ObstacleIdx(2,:); % Obstacle #2
        Obstacle_3 = ObstacleIdx(3,:); % Obstacle #3
        Obstacle_4 = ObstacleIdx(4,:); % Obstacle #4
        Obstacle_5 = ObstacleIdx(5,:); % Obstacle #5

        pos_1 = [Y(Obstacle_1(1),Obstacle_1(2)),X(Obstacle_1(1),Obstacle_1(2))];
        pos_2 = [Y(Obstacle_2(1),Obstacle_2(2)),X(Obstacle_2(1),Obstacle_2(2))];
        pos_3 = [Y(Obstacle_3(1),Obstacle_3(2)),X(Obstacle_3(1),Obstacle_3(2))];
        pos_4 = [Y(Obstacle_4(1),Obstacle_4(2)),X(Obstacle_4(1),Obstacle_4(2))];
        pos_5 = [Y(Obstacle_5(1),Obstacle_5(2)),X(Obstacle_5(1),Obstacle_5(2))];

        if k >= 1 && k <= 5
            if norm(pos_1-r_world)+r_o <= r_world
                Obstacle_1(1) = Obstacle_1(1)+n;
                Obstacle_1(2) = Obstacle_1(2)+n;
            end       
        end

        if k >= 1 && k <= 7
            if norm(pos_2-r_world)+r_o <= r_world
                Obstacle_2(1) = Obstacle_2(1)-1.4*n;
                Obstacle_2(2) = Obstacle_2(2)+0.4*n;
            end
        end

        if k >= 1 && k <= 7
            if norm(pos_3-r_world)+r_o <= r_world
                Obstacle_3(1) = Obstacle_3(1)-1.4*n;
                Obstacle_3(2) = Obstacle_3(2)+0.2*n;
            end
        end
        
        if k >= 4 && k <= 16
            if norm(pos_4-r_world)+r_o <= r_world
                Obstacle_4(1) = Obstacle_4(1)-n;
            end
        end
        
        if k >= 5 && k <= 14
            if norm(pos_5-r_world)+r_o <= r_world
                Obstacle_5(1) = Obstacle_5(1)+n;
                Obstacle_5(2) = Obstacle_5(2)+n;
            end
        end

        ObstacleIdx(1,:) = Obstacle_1;
        ObstacleIdx(2,:) = Obstacle_2;
        ObstacleIdx(3,:) = Obstacle_3;
        ObstacleIdx(4,:) = Obstacle_4;
        ObstacleIdx(5,:) = Obstacle_5;
        
    otherwise % Static obstacles
        ObstacleIdx = ObstacleIdx;
end



end

