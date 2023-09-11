%{
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - seed: seed of the mission
        - nb: num of drones in the swarm
        
    * Outputs:
        - seed_csv [seed, signed spoofing dev, attack drone, victim drone 1, victim drone 2, attack start time, attack duration]

%}

function drone_candidates(seed, nb, unsigned_dev)

    % iterate over missions (seed 200->350)
    currentFolder = pwd;
    addpath(fullfile(currentFolder, '../../examples/examples_swarm/'));
    addpath(fullfile(currentFolder,'../../fuzz/seed_generation'));
    root_f = fullfile(currentFolder,'../../'); 

    passing_csv = [root_f 'fuzz/tmp_files/passing_dire' num2str(seed) '.csv'];  
    seed_csv = [root_f 'fuzz/seed_generation/seedpool' num2str(seed) '.csv'];

%     Delete previous temp files
    if isfile(seed_csv)
        delete(seed_csv)
    end
    
    % iterate drone att_id
    for att_id=1:1
        % randomly select two other drones, except the att_id 
    
        if att_id == 1
            vic_id1 = 2;
            vic_id2 = nb;
        elseif att_id == nb
            vic_id1 = 1;
            vic_id2 = nb-1;
        else
            vic_id1 = att_id-1;
            vic_id2 = att_id+1;
        end

        start_t = 0;
        dur = 40;
        passing_csv = [root_f 'fuzz/tmp_files/passing_dire' num2str(seed) '.csv'];  
        no_attack_dire_csv = [root_f 'fuzz/tmp_files/no_attack_dire_info' num2str(seed) '.csv'];  
        pos_dire_csv = [root_f 'fuzz/tmp_files/pos_dire_info' num2str(seed) '.csv'];  
        neg_dire_csv = [root_f 'fuzz/tmp_files/neg_dire_info' num2str(seed) '.csv']; 
        posY_csv = [root_f 'fuzz/tmp_files/posY' num2str(seed) '.csv']; 
        posYpos_csv = [root_f 'fuzz/tmp_files/posYpos' num2str(seed) '.csv']; 
        posYneg_csv = [root_f 'fuzz/tmp_files/posYneg' num2str(seed) '.csv']; 
        dist_csv = [root_f 'fuzz/tmp_files/dist' num2str(seed) '.csv'];
       
        if isfile(passing_csv)
            delete(passing_csv)
        end
        if isfile(no_attack_dire_csv)
            delete(no_attack_dire_csv)
        end
        if isfile(pos_dire_csv)
            delete(pos_dire_csv)
        end
        if isfile(neg_dire_csv)
            delete(neg_dire_csv)
        end
        if isfile(posY_csv)
            delete(posY_csv)
        end
        if isfile(posYpos_csv)
            delete(posYpos_csv)
        end
        if isfile(posYneg_csv)
            delete(posYneg_csv)
        end
        if isfile(dist_csv)
            delete(dist_csv)
        end
                
    
        
        % Run without and with the attack   
        example_olfati_saber(start_t, dur, att_id, vic_id1, vic_id2, 0, seed, no_attack_dire_csv, posY_csv, dist_csv); 
            
%       postive deviation situation
        example_olfati_saber(start_t, dur, att_id, vic_id1, vic_id2, unsigned_dev, seed, pos_dire_csv, posYpos_csv, dist_csv); 

%       negative deviation situation 
        example_olfati_saber(start_t, dur, att_id, vic_id1, vic_id2, (-1)*unsigned_dev, seed, neg_dire_csv, posYneg_csv, dist_csv); 

        % Read the file and record the direction passing the obstacle
        no_attack_dire_mat = readmatrix(no_attack_dire_csv);
        pos_dire_mat = readmatrix(pos_dire_csv);
        neg_dire_mat = readmatrix(neg_dire_csv);
        posYpos_mat = readmatrix(posYpos_csv);
        posYneg_mat = readmatrix(posYneg_csv);
        % Find the early index that the victim drone's y axis is larger than 93
        threY = 93;
        dire_list = zeros(3,7); % For each seed, record the direction
        dire_list(1,1) = seed; % Passing direction when no attack
        dire_list(2,1) = seed; % Passing direction when positive deviation
        dire_list(3,1) = seed; % Passing direction when negative deviation
        dire_list(1,2) = 0;
        dire_list(2,2) = unsigned_dev;
        dire_list(3,2) = (-1)*unsigned_dev;
        seed_mat = [];    

        for i=1:nb
              if (i~=att_id)
              % For each drone, find the index of its first appearance - when no attack
                tmp = min(find(no_attack_dire_mat(:, 1)==i));
                if tmp
                    % Return the the y-axis of each drone
                    dire1 = (no_attack_dire_mat(tmp,3)-150)/abs(no_attack_dire_mat(tmp,3)-150);
                    dire_list(1, i+2) = dire1;
                else
                    dire1 = 0;
                    dire_list(1, i+2) = dire1;
                end
                % For each drone, find the index of its first appearance - when positive deviation 
                tmp = min(find(pos_dire_mat(:, 1)==i));
                if tmp 
                    dire2 = (pos_dire_mat(tmp,3)-150)/abs(pos_dire_mat(tmp,3)-150);
                    dire_list(2, i+2) = dire2;
                else
                    dire2 = 0;
                    dire_list(2, i+2) = dire2;
                end
                % For each drone, find the index of its first appearance - when negative deviation
                tmp = min(find(neg_dire_mat(:, 1)==i));
                if tmp
                    dire3 = (neg_dire_mat(tmp,3)-150)/abs(neg_dire_mat(tmp,3)-150);
                    dire_list(3, i+2) = dire3;
                else
                    dire3 = 0;
                    dire_list(3, i+2) = dire3;
                end
                % compare the direction - only record the different drones
                % if the positive deviation is different with no attack one
                if ((dire1 ~= dire2) && (abs(dire1)==1) && (abs(dire2)==1))
                   vic1_id = i;
                   %  -------- Find victim drone 2 ----------
                   % Find the early index that the victim drone's y axis is larger than threY 93
                   max_idx = min(find(posYpos_mat(:, vic1_id)>threY)); 
                   % iterate until max_idx, record the number of times that other drones fall behind the victim drone
                    fall_behind = 1:1:nb; % 
                    behind_arr = zeros(1, nb);
                    for k =1: max_idx
                        behind_idx = find(posYpos_mat(k, :)<posYpos_mat(k, vic1_id));
                        if (behind_idx)
                            for j=1:length(behind_idx)
                                if (behind_idx(j)~=att_id)
                                behind_arr(behind_idx(j))= behind_arr(behind_idx(j))+ 1;
                                end
                            end
                        end
                    end
                    % If the drone is fall falling behind over 50% of the time.
                    fall_behind = [fall_behind;behind_arr];
                    fall_behind = fall_behind';
                    fall_behind = sortrows(fall_behind,2,"ascend"); % prioritize the nearest drone

                    if max_idx
                        fall_idx = find(fall_behind(:, 2)>(max_idx/2));
                        if fall_idx
                            vic2_id_arr = fall_behind(fall_idx, 1);
                        else
                            disp("continue")
                            continue
                        end

                        for q = 1: length(vic2_id_arr)
                            seed_mat = [seed_mat; seed, unsigned_dev, att_id, vic1_id, vic2_id_arr(q), start_t, dur]; 
                        end
                    end
                end
             

                % if the negative deviation is different with no attack one
                if ((dire1 ~= dire3) && (abs(dire1)==1) && (abs(dire2)==1))
                   vic1_id = i;
                   %  -------- Find victim drone 2 ----------
                   % Find the early index that the victim drone's y axis is larger than threY 93
                   max_idx = min(find(posYneg_mat(:, vic1_id)>threY)); 
                   % iterate until max_idx, record the number of times that other drones fall behind the victim drone
                    fall_behind = 1:1:nb; % 
                    behind_arr = (-1)*ones(1, nb);
                    for k =1: max_idx
                        behind_idx = find(posYneg_mat(k, :)<posYneg_mat(k, vic1_id));
                        if (behind_idx)
                            for j=1:length(behind_idx)
                                if (behind_idx(j)~=att_id)
                                behind_arr(behind_idx(j))= behind_arr(behind_idx(j))+ 1;
                                end
                            end
                        end
                    end
                    % If the drone is fall falling behind over 50% of the time.

                    fall_behind = [fall_behind;behind_arr];
                    fall_behind = fall_behind';
                    fall_behind = sortrows(fall_behind,2,"ascend"); % prioritize the nearest drone
                    if max_idx
                        fall_idx = find(fall_behind(:, 2)>(max_idx/2));
                        if fall_idx
                            vic2_id_arr = fall_behind(fall_idx, 1);
                        else
                            disp("continue")
                            continue
                        end
                        for q = 1: length(vic2_id_arr)
                            seed_mat = [seed_mat; seed, (-1)*unsigned_dev, att_id, vic1_id, vic2_id_arr(q), start_t, dur]; 
                        end
                    end
                end
              end
        end
          
           writematrix(dire_list, passing_csv, 'Delimiter', ',', 'WriteMode', 'append');
           writematrix(seed_mat, seed_csv, 'Delimiter', ',', 'WriteMode', 'append');

    end
end

