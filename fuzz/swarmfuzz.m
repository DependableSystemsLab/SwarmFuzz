function swarmfuzz(seedStart, seedEnd, dev, nb)

    c_obs = [50;150]; % hard code the location of the obstacle
    max_ite = 20; % Maximum number of iterations
    dur_t = 0.1; % Start value

    % -------- 0. Add paths
    currentFolder = pwd;
    addpath(fullfile(currentFolder, '../examples/examples_swarm/'));
%     addpath(fullfile(currentFolder,'../../fuzz/prepare'));
%     addpath(fullfile(currentFolder,'../../fuzz/search'));
%     addpath(fullfile(currentFolder,'../../fuzz/seed_generation'));
    root_f = fullfile(currentFolder,'../');
    
    for seed = seedStart:seedEnd
        
        % temp files
        pos_csv = [root_f 'fuzz/tmp_files/attack_pos' num2str(seed) '.csv'];
        dist_csv = [root_f 'fuzz/tmp_files/dist_obs' num2str(seed) '.csv'];
        col_csv = [root_f 'fuzz/tmp_files/nb_col' num2str(seed) '.csv'];
        info_csv = [root_f 'fuzz/tmp_files/col_info' num2str(seed) '.csv'];
        parent_csv = [root_f 'fuzz/tmp_files/parent' num2str(seed) '.csv'];
        neigh_csv = [root_f 'fuzz/tmp_files/neighbor' num2str(seed) '.csv'];

        % output files
        ite_csv = [root_f 'fuzz/search/iteration' num2str(seed) '.csv'];
        cond_csv = [root_f 'fuzz/search/condition' num2str(seed) '.csv'];
        % may comment this later
        dire_csv = [root_f 'fuzz/prepare/dire' num2str(seed) '.csv'];
%         seedpool_csv = [root_f 'fuzz/seed_generation/pool' num2str(seed) '.csv'];
        seedpool_csv = [root_f 'fuzz/seedpools/pool' num2str(seed) '.csv']; % for computecanada version
        f_out = [root_f 'fuzz/attResults/att_results' num2str(seed) '.csv'];
        
        % Delete previous temp files
        if isfile(pos_csv)
            delete(pos_csv);
        end
        if isfile(dist_csv)
            delete(dist_csv);
        end
        if isfile(col_csv)
            delete(col_csv)
        end
        if isfile(info_csv)
            delete(info_csv)
        end
        if isfile(parent_csv)
            delete(parent_csv)
        end
        if isfile(neigh_csv)
            delete(neigh_csv)
        end

        %%      -------- 1. Preparation ---------
        disp(["********** No attack. Running seed:" num2str(seed) "**********"]);
        example_vasarhelyi(0, 0, 0, 0, 0, seed, pos_csv, dist_csv, col_csv, info_csv); % (start_t, dur, att_id, vic_id, dev_y, sed)
        [start_t, pos_att, dist_obs, final_dire] = prepare(nb, c_obs, pos_csv, dist_csv, dire_csv, col_csv, seed); % ( pos_csv, nb, c_obs, dist_csv, dire_csv)
    
        %%  ---------2. Generate seedpool ---------
        if (start_t > -1) % generate seedpool only if no collisions without attack
            disp("*********** Swarm vulnerability graph generator **********");
            flag = seed_gen(nb, start_t, pos_att, dist_obs, final_dire, seedpool_csv); % (nb, start_t, pos_att, dist_obs, seedpool)
        else
            disp(["No seedpool for seed " num2str(seed)])
        end     
        %% ----------3. Gradient descent ----------
        if isfile(seedpool_csv) %          if seedfile exist, then continue, or break
            disp("*********** Gradient descent ***********");
            search_grad(seedpool_csv, dev, max_ite, f_out, dur_t, seed, pos_csv, dist_csv, col_csv, info_csv, parent_csv, neigh_csv, ite_csv, cond_csv);
        end

        % Delete previous temp files
        if isfile(pos_csv)
            delete(pos_csv);
        end
        if isfile(dist_csv)
            delete(dist_csv);
        end
        if isfile(col_csv)
            delete(col_csv)
        end
        if isfile(info_csv)
            delete(info_csv)
        end
        if isfile(parent_csv)
            delete(parent_csv)
        end
        if isfile(neigh_csv)
            delete(neigh_csv)
        end
    end
end
