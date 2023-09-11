%{
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - seed: seed of the mission
        - att_id:
        - vic1_id:
        - vic2_id:
        
    * Outputs:
        - gpsParam_csv [success, seed, signed spoofing dev, attack drone, victim drone 1, victim drone 2, attack start time, attack duration]

%}

function gps_params(seed)
    % -------- 0. Add paths
    currentFolder = pwd;
    addpath(fullfile(currentFolder, '../../examples/examples_swarm/'));
    addpath(fullfile(currentFolder,'../../fuzz/prepare'));
    addpath(fullfile(currentFolder,'../../fuzz/search'));
    addpath(fullfile(currentFolder,'../../fuzz/seed_generation'));
    root_f = fullfile(currentFolder,'../../');
 
    seed_csv = [root_f 'fuzz/seed_generation/seedpool' num2str(seed) '.csv'];
    gpsParam_csv = [root_f 'fuzz/search/gpsParam_csv' num2str(seed) '.csv'];
    paramTmp_csv = [root_f 'fuzz/search/paramTmp_csv' num2str(seed) '.csv'];
    
    seedpool = readmatrix(seed_csv);
    rows = size(seedpool, 1);
    if isfile(gpsParam_csv)
        delete(gpsParam_csv)
    end
    if isfile(paramTmp_csv)
        delete(paramTmp_csv)
    end
    for row =1:rows
        
        success = 0;
        flag = 1; % means when dur_min, d_min <0, then usually binary search
        signed_dev = seedpool(row, 2);
        att_id = seedpool(row, 3);
        vic1_id = seedpool(row, 4);
        vic2_id = seedpool(row, 5);
        start_t = seedpool(row, 6);
        dur_init = seedpool(row, 7);
        
        % Run the experiment and return the initial dist between victim
        % drones under no attack.
         
        % dist_csv only returns the dist for the whole mission
        no_attack_dire_csv = [root_f 'fuzz/tmp_files/no_attack_dire_info' num2str(seed) '.csv']; 
        posY_csv = [root_f 'fuzz/tmp_files/posY' num2str(seed) '.csv'];
        dist_csv = [root_f 'fuzz/tmp_files/dist' num2str(seed) '.csv'];
        if isfile(no_attack_dire_csv)
            delete(no_attack_dire_csv)
        end
        if isfile(posY_csv)
            delete(posY_csv)
        end
        if isfile(dist_csv)
            delete(dist_csv)
        end
        example_olfati_saber(start_t, dur_init, att_id, vic1_id, vic2_id, 0, seed, no_attack_dire_csv, posY_csv, dist_csv); 
        if isfile(dist_csv)
        dist_mat = readmatrix(dist_csv);
        min_abs = min(abs(dist_mat(:, 2)));
        min_idx = find(abs(dist_mat(:, 2))==min_abs);
        d_min = dist_mat(min_idx, 2);
        if d_min >0
            flag = 0;
        end
        delete(dist_csv)
        else
            continue
        end
   
        example_olfati_saber(start_t, dur_init, att_id, vic1_id, vic2_id, signed_dev, seed, no_attack_dire_csv, posY_csv, dist_csv); 
        if isfile(dist_csv)
            dist_mat = readmatrix(dist_csv);
            min_abs = min(abs(dist_mat(:, 2)));
            min_idx = find(abs(dist_mat(:, 2))==min_abs);
            d_max = dist_mat(min_idx, 2);
            delete(dist_csv)
        else
            continue
        end

        % range of the duration
        dur_min = 0;
        dur_max = dur_init;
        ite_num = 0;
        dur = 0;

        % direction of min and max distance
        d_min_dire = d_min/abs(d_min);
        d_max_dire = d_max/abs(d_max);

        time_mat = [0, 0, 0; 0, 0, 0];
        % Collision is only possible if the directions are opposite
        while((dur_max>=dur_min+0.005)&& ((d_min_dire+d_max_dire) == 0))
           
            dur_mid = (dur_min + dur_max)/2;
            example_olfati_saber(start_t, dur_mid, att_id, vic1_id, vic2_id, signed_dev, seed, no_attack_dire_csv, posY_csv, dist_csv); 
            if isfile(dist_csv)
                dist_mat = readmatrix(dist_csv);
                min_abs = min(abs(dist_mat(:, 2)));
                min_idx = find(abs(dist_mat(:, 2))==min_abs);
                d_mid = dist_mat(min_idx, 2);
                delete(dist_csv)
                ite_num = ite_num+1;
    
                % if the distance between two drones is smaller than 0.5m.
                if d_mid > 0.5
                    if flag
                        dur_max = dur_mid;
                        d_max = d_mid;
                    else
                        d_min = d_mid;
                        dur_min = dur_mid;
                    end
                elseif d_mid < -0.5
                    if flag
                        d_min = d_mid;
                        dur_min = dur_mid;
                    else
                        dur_max = dur_mid;
                        d_max = d_mid;
                    end
                else
                    success = 1;
                    dur = dur_mid;
                    break
                end
                ite_num = ite_num+1;
                tmp_mat = [ite_num, att_id, vic1_id, vic2_id, dur_min, d_min, dur_max, d_max, dur_mid];
                writematrix(tmp_mat, paramTmp_csv, 'Delimiter', ',', 'WriteMode', 'append');
            else
                continue
            end
        end
        gps_mat = [success, seed, signed_dev, att_id, vic1_id, vic2_id, start_t, dur];
        writematrix(gps_mat, gpsParam_csv, 'Delimiter', ',', 'WriteMode', 'append');
    end

end

