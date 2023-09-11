% TODO: 1. Select false data value
%       2. Select the time that the drone pass before the obstacle
%       3. Apply graph in x-axis also
function swarmfuzzbinary(seedStart, seedEnd, unsigned_dev, nb)

    % -------- 0. Params setting ----------
    % When changing the number of drones in the swarm, remember to modify
    % - p_swarm.nb_agents
    % - nb in Swarm.m about color

   
    % -------- 0. Add paths
    currentFolder = pwd;
    addpath(fullfile(currentFolder, '../../examples/examples_swarm/'));
    addpath(fullfile(currentFolder,'../../fuzz/search'));
    addpath(fullfile(currentFolder,'../../fuzz/seed_generation'));
    root_f = fullfile(currentFolder,'../../');
%     seed_csv = [root_f 'fuzz/seed_generation/seedpool' num2str(seed) '.csv'];
    for seed = seedStart:seedEnd
    %          -------- 1. Generate the seed pool. Attack and victim drone 1&2 ---------
        % only generate the seedpool when the seeds are not generated
        drone_candidates(seed, nb, unsigned_dev)

        %  ---------2. Binary search ---------
        gps_params(seed)
    end


end
