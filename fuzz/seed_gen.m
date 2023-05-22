%{
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - nb: Number of drones in the swarm
        - start_t: Initial attack start time
        - pos_att: Drones' location at initial attack start time
        - dist_obs: The min distance between each drone and the obstacle
        - final_dire: Direction of each swarm member when it passes the obstacle ### We can use gradient to exclude this scenario
        
    * Outputs:
        - seedpool: Generated seedpool for fuzzing (csv)
        - flag: 1 - there is a valid seedpool; 0 - no valid seedpool ? 

%}

function flag = seed_gen(nb, start_t, pos_att, dist_obs, final_dire, seedpool)
    
    %% Initializaion
    nb_v = 2; % number of victim drones selected for seed scheduling ? 
    nb_t = 2; % number of target drones selected for seed scheduling ? 
    dist_thre = 5; % threshold for the distance difference between top nb_v victim drones
    pool = zeros(nb_v*nb_t, 4); % initialize the seedpool
    flag = -1;
    
    %% Construct swarm vulnerability graph
    
    % ***** 1. Identify the target direction of the victim drones when under attacks *****
    % The attack goal for the direction of victim drones - opposite with the drones' direction in the no-attack scenario
    goal = -final_dire;
    
    % ***** 2. Decide the initial victim drones *****
    % Choose the top nb_v(e.g., 2) drones with smallest distance as the initial victim drones
    [dist_vic_tmp, vic_id_tmp] = mink(dist_obs,nb_v); 
    % If the distance difference between the top nb_v drones is > dist_thre, then the drop the drone with larger distance
    if dist_vic_tmp(2)>(dist_vic_tmp(1)+dist_thre)
        vic_id_tmp = vic_id_tmp(1);
        nb_v = 1;
    end
    
    % ***** 3. Build the graph and compute the page rank centrality *****
    
    % *** Consider 2 scenarios.
    %        dev = 1 - the target drone deviates to its right side.
    %        dev = -1 - the target drone deviates to its left side. ***
    dev = [1, -1]; 
    A = zeros(2, nb, nb); % graphs in 2 scenarios
    flags = zeros(2, 1); % validity for graph in 2 scenarios
    pgrank_tar = zeros(2, nb); % pagerank centrality for target drones in 2 scenarios
    pgrank_vic = zeros(2, nb); % pagerank centrality for victim drones in 2 scenarios
    
    for i = 1: length(dev) % for each deviation direction
        [A(i, :, :), flags(i, :)] = build_graph(dev(i), goal, pos_att, nb); % build the graph A
        % If the graph exists, compute the pagerank centrality
        if flags(i, :) == 1
            % Graph for the influential nodes (target drones)
            G_tar = digraph(squeeze(A(i, :, :)));
            %  plot(G, 'NodeLabel', {'1', '2', '3', '4', '5'})
            % page rank algorithm
            pgrank_tar(i, :) = centrality(G_tar, 'pagerank', 'Importance', G_tar.Edges.Weight);
            % Graph for the nodes being influenced (victim drones)
            G_vic = digraph(squeeze(A(i, :, :))');
            pgrank_vic(i, :) = centrality(G_vic, 'pagerank', 'Importance', G_vic.Edges.Weight);
        end
    end
    
    %% Seed scheduling
    valid_idx = find(flags == 1);
    % if both left and right deviation help with the attack goal
    % we should merge the tar-vic drone pairs in two scenarios above.
    if length(valid_idx) == 2
        flag = 1;
        % we first select the most promising victim drone, i.e., closet to
         % the obstacle. 
         % Then, we compare the pg_rank centrality of this
         % victim drone in two scenarios, to decide what kind of deviation
         % should be prioritized.
         for i = 1:nb_v
             idx = vic_id_tmp(i); % index of the initial victim drones
             tmp_pg_ranks_tar1 = squeeze(pgrank_tar(1,:)');
             tmp_pg_ranks_tar2 = squeeze(pgrank_tar(2,:)');
             tmp_pg_ranks_tar1(idx) = -1; 
             tmp_pg_ranks_tar2(idx) = -1;
             if (pgrank_vic(1, idx)>pgrank_vic(2, idx)) % right deviation is more influential to this victim drone
                 [tmp2, target_id] = maxk(tmp_pg_ranks_tar1,nb_t); % thus, select 2 target drones with the highest score with right deviation
                 f = [1;1];
             elseif (pgrank_vic(1, idx)<pgrank_vic(2, idx)) % left deviation is more influential to this victim drone
                 [tmp2, target_id] = maxk(tmp_pg_ranks_tar2,nb_t); % thus, select 2 target drones with the highest score with left deviation
                 f = [-1;-1];
             else % if the influence for right and left deviation is the same
                 % sum the total score of top 2 target drones in each scenario 
                 [tmp2, target_id1] = maxk(tmp_pg_ranks_tar1,nb_t);
                 [tmp3, target_id2] = maxk(tmp_pg_ranks_tar2,nb_t);
                 % select the deviation direction where the total score is higher
                 if(sum(tmp2)>sum(tmp3))
                     target_id = target_id1;
                     f = [1;1];
                 else
                     target_id = target_id2;
                     f = [-1;-1];
                 end
             end
             victim_id = [idx;idx]; % for each victim drone, we have 2 target drones 
             time = [start_t; start_t];
             pool(2*i-1:2*i, :) = [target_id victim_id f time]; % seedpool
         end
         % delete any zero rows
         pool = pool(any(pool,2),:);
         writematrix(pool, seedpool, 'Delimiter', ',');
         disp(pool)
    
    % if only one direction of deviation is valid
    elseif length(valid_idx) == 1
        flag = 1;
        for i = 1:nb_v
             idx = vic_id_tmp(i);
             tmp_pg_ranks_tar = squeeze(pgrank_tar(valid_idx, :)');
             tmp_pg_ranks_tar(idx) = -1;
             [tmp2, target_id] = maxk(tmp_pg_ranks_tar,nb_t);
             if valid_idx == 1
                 f = [1;1];
             else
                 f = [-1;-1];
             end
           
             victim_id = [idx;idx];
             time = [start_t; start_t];
             pool(2*i-1:2*i, :) = [target_id victim_id f time];
        end
         % delete any zero rows
         pool = pool(any(pool,2),:);
         writematrix(pool, seedpool, 'Delimiter', ',');
         disp(pool)
        
    else
       flag = 0;   
    end
        
     
end










