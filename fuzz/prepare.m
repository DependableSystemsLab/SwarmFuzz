%{
    * Phase 1: Prepare for the information(i.e., passing direction, initial attack time, relative distance) for phase 2 
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - pos_csv: Location records for each swarm member in a no-attack mission
        - nb: Number of drones in the swarm
        - c_obs: Location of the obstacle
        - dist_csv: Record of the distances between each drone and the obstacle in a no-attack mission
        - dire_csv: (empty file) Direction of each swarm member when it passes the obstacle ### We can use gradient to exclude this scenario 
    * Outputs:
        - start_t: Initial attack start time
        - pos_att: Drones' location at initial attack start time
        - dist_obs: The min distance between each drone and the obstacle
        - final_dire: Direction of each swarm member when it passes the obstacle

%}

function [start_t, pos_att, dist_obs, final_dire] = prepare(nb, c_obs, pos_csv, dist_csv, dire_csv, col_csv, seed)
    
    currentFolder = pwd;
    root_f = fullfile(currentFolder,'../');
    addpath(fullfile(currentFolder, '../parameters/'));
    run('param_sim');
    % In dire_csv, "1" means the swarm member passes the obstacle from the right;
    % "-1" means from the left
    r = 1; 
    l = -1;
    
    % CSV files to store the properties of different type of missions;
    % Read if there is previous files storing the mission   
    proper_csv = [root_f 'fuzz/tmp_files/mission_property.csv'];
    if isfile(proper_csv)
        proper_mat = readmatrix(proper_csv);
    else
        proper_mat = zeros(1,3);
    end
    new_proper = zeros(1,3); % used to record the property of current mission. seed | collision | same_direction
    new_proper(1) = seed;

    %% Identify if there is collisions even without attack
    col_mat = readmatrix(col_csv);
    col_idx = find(col_mat(:, 2)==1);
    if col_idx
        new_proper(2) = 1;
    end
    
    %% Direction of drones when passing the obstacle - For building the graph
    pos_mat = readmatrix(pos_csv);
    final_dire = zeros(1, nb);
    early_idx = p_sim.end_time*100; % the time before the victim drone passes the obstacle
    for i = 1:nb
        % find the first point where drone reaches the same x as the obstacle
        idx = find(pos_mat(:, 2*i)>c_obs(1),1);
        if isempty(idx) 
            idx = p_sim.end_time*100;
        end
        early_idx = min(idx, early_idx);
        if(pos_mat(idx, 2*i+1)>c_obs(2))
            final_dire(i) = r;
        else
            final_dire(i) = l;
        end
    end
    % identify whether the passing direction of all drones is the same
    if (abs(sum(final_dire)) == nb)
        % if the passing direction of all drones is the same, won't consider
%         start_t = -1;
%         pos_att = -1;
%         dist_obs = -1;
        new_proper(3) = 1;
        disp("***** The passing direction of all drones is the same, we'd better drop this. *****")
    end
    
      % drop the collision missions
    if (new_proper(2) == 1)
            % if collisions occur without attack, won't consider
            start_t = -1;
            pos_att = -1;
            dist_obs = -1;
            final_dire = zeros(1, nb);
            % find the property recording the same mission
            same_mission_idx = find(proper_mat(:, 1) ==new_proper(1));
           if same_mission_idx
                proper_mat(same_mission_idx(1), :) = new_proper;
                while(length(same_mission_idx)>1)
                    proper_mat(same_mission_idx(2), :) = [];
                    same_mission_idx = find(proper_mat(:, 1) ==new_proper(1));
                end
            else
                proper_mat = cat(1, proper_mat, new_proper);
            end
            % delete any zero rows
            proper_mat(~any(proper_mat,2), :) = [];
            writematrix(proper_mat, proper_csv, 'Delimiter', ',');
            disp("***** Collisions occur without attack, won't consider. *****")
            return
     end

    %% Initial attack start time - minimal sum - For fuzzing
    % Find out the the time before the victim drone passes the obstacle (i.e., early_idx)
    rows = early_idx;
    dist_mat = zeros(rows, 2);
    
    for i = 1:rows
        pos_row = reshape(pos_mat(i, 2:end), [2,nb]); % each drone's location at time i. [x1, x2, ..., x_nb; y1, y2, ..., y_nb]
        pos_row = pos_row';
        dist_row = pdist(pos_row); % pairwise distance between drones
        dist_mat(i, :) = [pos_mat(i,1) sum(dist_row)]; % sum of each pairwise distance between drones at time i
    end  
    start_idx = find(dist_mat(:,2) == min(dist_mat(:,2)) ); % find the time where the distance sum is the smallest (drones are closet to each other)
    start_t = max(pos_mat(start_idx), 0.01); % for gradient descent
    
    %% Drones' location at initial attack start time - For building the graph
    row_id = start_t*100;
    pos_mat = readmatrix(pos_csv);
    pos_att = pos_mat(floor(row_id), 2:(2*nb+1)); % the distance between each drone and the obstacle at start_t
    pos_att = reshape(pos_att, [2,nb]);
    
    %% Min dist between the drone and the obstacle in the no-attack mission - For seed scheduling
    dist_obs = zeros(nb,1);
    dist_all = readmatrix(dist_csv);
    for i = 1:nb % for each drone
        idx = find(dist_all(:, 2) == i); 
        dist_obs(i) = min(dist_all(idx, 3));
    end
    
    %% Write the mission type into csv files, delete zero and repetive number
    % find the property recording the same mission
    same_mission_idx = find(proper_mat(:, 1) ==new_proper(1));
    if same_mission_idx
        proper_mat(same_mission_idx(1), :) = new_proper;
        while(length(same_mission_idx)>1)
            proper_mat(same_mission_idx(2), :) = [];
            same_mission_idx = find(proper_mat(:, 1) ==new_proper(1));
        end
        
    else
        proper_mat = cat(1, proper_mat, new_proper);
    end
    % delete any zero rows
    proper_mat(~any(proper_mat,2), :) = [];
    writematrix(proper_mat, proper_csv, 'Delimiter', ',');
    
end