%{
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - dev: Direction of deviation. 1 - right, -1 - left
        - goal: Target direction of drones under attack
        - pos_att: Drones' location at initial attack start time
        - nb: Number of drones in the swarm
        
    * Outputs:
        - A: Generated swarm vulnerability graph
        - flag: Existance of the graph. 1 - exist; 0 - doesn't exist

%}

function [A, flag] = build_graph(dev, goal, pos_att, nb)
    % Initilization
    v = ones(1, nb);
    A = ones(nb) - diag(v); % initilize the adjecent matrix
    A_zero = zeros(nb, nb);
    
    % 
    for i = 1:nb
        for j = 1:nb
            if (i == j)
                continue % victim drone and attack drone should be different
            end
    
            if (dev == goal(j)) % devation has positive influence
                % ***   E.g., the attack drone deviates to its right side (i.e., dev = 1). 
                %       This helps other drones also deviate to its right side,
                %       regardless of the relative location between the attack drone and other drones. (see papers)
                %       Thereby, if the target direction of the victim drone is also right (i.e., goal(j) = 1),
                %       the deviation of the attack drone has positive influence 
                %       and we add an edge with the calculated weight in the graph. ***
                A(j, i) = abs((pos_att(2,i)-pos_att(2,j))/(pos_att(1,i)-pos_att(1,j))); % calculate the weight
            else % deviation has no positive influnence
                A(j, i) = 0;
            end
        
        end
     end
    
     if(A == A_zero) % Impossible
         flag = 0;
     else
         flag = 1;
     end

end

