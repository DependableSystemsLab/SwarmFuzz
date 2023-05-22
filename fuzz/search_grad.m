%{
    * Create the swarm vulnerability graph and generate seedpool for fuzzing
    * Inputs:
        - pool_f: csv file to store the seed for fuzzing
        - val: gps spoofing deviation
        - max_ite: maximum number of gradient descent search iteration
        - f_out: csv file to store the results of fuzzing
        - dur: initial value for gps spoofing attack
        - seed: the seed for random number generator to create the swarm
        mission
        - pos_csv: csv file recording each drone's x_y location at each timestamp
        - dist_csv: csv file recording the distance between each agent and the obstacle
        - col_csv: csv file recording the number of collisions betweent the victim drone and the obstacle
        - info_csv: csv file recording the distance between the victim
        drone and the obstacle at each timestamp
        - parent_csv: csv file recording the collision information at x point (used to calculate the gradient)
        - neigh_csv: csv file recording the collision information at (x + delta x) point (used to calculate the gradient)
        - ite_csv: csv file to store the results for each gradient descent
        iteration
        - cond_csv: record the gradient descent condition. The derivation is only valid at the certain point
        
    * Outputs:
        - seedpool: Generated seedpool for fuzzing (csv)
        - flag: 1 - there is a valid seedpool; 0 - no valid seedpool ? 

%}
function search_grad(pool_f, val, max_ite, f_out, dur, seed, pos_csv, dist_csv, col_csv, info_csv, parent_csv, neigh_csv, ite_csv, cond_csv)
    
    currentFolder = pwd;
    addpath(fullfile(currentFolder, '../examples/examples_swarm/'));
    addpath(fullfile(currentFolder, '../parameters/'));

% ---------- 1. Read the initial seeds and iterate them one by one -------
    seedpool = readmatrix(pool_f);
    [rows, cols] = size(seedpool);
    run('param_sim');
    disp(p_sim.end_time);
    
    % Global init
    attack_start = 0;
    duration = 0;
    attack_id = 0;
    victim_id = 0;
    deviation = 0;
    final_col = 0;
    out_mat = zeros(rows, 8);
    init_fitness = 0;
    
    % Delete previous tmp files

    if isfile(ite_csv)
        delete(ite_csv)
    end
    if isfile(cond_csv)
        delete(cond_csv)
    end
    
    % delta x,y
    st_del = 0.01;
    t_del = 0.01;
    lr = 0;
    
    for row = 1:rows
        % Initial seeds
        t_init = dur;
        att_id = seedpool(row, 1);
        vim_id = seedpool(row, 2);
        dev_y = val*seedpool(row, 3);
        st_init = seedpool(row, 4);
        ite = 0;
  
        while (ite<max_ite)
            % Parent point
            disp(["Seed: " num2str(seed) "Parent: " num2str(st_init) ", " num2str(t_init)])
            example_vasarhelyi(st_init, t_init,att_id,vim_id, dev_y, seed, pos_csv, dist_csv, col_csv, info_csv)
            cal_summary(col_csv, info_csv, parent_csv);
            % [start_t, dur, col_flag, col_time, dist_obs_min];
            record = readmatrix(parent_csv);
            nb_col = record(3);
            fitness = record(5);
            if ite == 0
                init_fitness = fitness;
            end
            
            min_time = record(4);
            ite_mat = [fitness,lr, st_init, t_init];
            writematrix(ite_mat, ite_csv, 'Delimiter', ',', 'WriteMode', 'append');
            
            
            if (nb_col == 1)
                disp(["Seed: " num2str(seed) "At " num2str(record(4)) " collides, start time = " num2str(record(1)) ", duration = " num2str(record(2))]);
                attack_start = record(1);
                duration = record(2);
                attack_id = att_id;
                victim_id = vim_id;
                deviation = dev_y;
                break;
            end
            % neighbor for differential derivates
            % f(x+st_del, y)
            st = st_init+ st_del;
            t = t_init;

%             disp(["Neighbor: " num2str(st) ", " num2str(t)])
            example_vasarhelyi(st, t, att_id, vim_id, dev_y, seed, pos_csv, dist_csv, col_csv, info_csv);
            cal_summary(col_csv, info_csv, neigh_csv);
            % [start_t, dur, col_flag, col_time, dist_obs_min];
            new_record = readmatrix(neigh_csv);
            new_nb_col = new_record(3);
            x_fitness1 = new_record(5);

            if (new_nb_col == 1)
                attack_start = new_record(1);
                duration = new_record(2);
                attack_id = att_id;
                victim_id = vim_id;
                deviation = dev_y;  
                break;
            end
            
            % f(x, y+t_del)
            st = st_init;
            t = t_init+t_del;

%             disp(["Neighbor: " num2str(st) ", " num2str(t)])
            example_vasarhelyi(st, t, att_id, vim_id, dev_y, seed, pos_csv, dist_csv, col_csv, info_csv);
            cal_summary(col_csv, info_csv, neigh_csv);
            new_record = readmatrix(neigh_csv);
            new_nb_col = new_record(3);
            y_fitness1 = new_record(5);

            if (new_nb_col == 1)
                attack_start = new_record(1);
                duration = new_record(2);
                attack_id = att_id;
                victim_id = vim_id;
                deviation = dev_y;  
                break;
            end
            dst = (x_fitness1-fitness)/(st_del);
            dt = (y_fitness1-fitness)/(t_del);  
            if dst == 0
                dst = 1e-06;
            end
            if dt == 0
                dt = 1e-06; % we set eps positive because we don't want this to break
            end
               
            st_tmp = st_init;
            t_tmp = t_init;

            
            % Calculate lr
            loss = fitness - 0.5;
            k = max(abs(dst), abs(dt));
             
            lr = loss/(2*k*k);

            st_init = st_tmp - lr*dst; % w/o momentum
            t_init = max(t_tmp - lr*dt,0); % w/o momentum
            
            % if the calculated time is too long, give up
            if ((st_init +t_init)>p_sim.end_time) || (st_init <0) 
                break;
            end
        
            ite = ite+1;
            
            % calculate whether it's possible or not
            max_improve = -(min_time-st_tmp)*dt;
            % record the condition, the derivation is only valid at the certain point
            cond_mat = [min_time, dt, max_improve, loss, dst, st_tmp, t_tmp];
            writematrix(cond_mat, cond_csv,'Delimiter', ',', 'WriteMode', 'append');
            if (ite>5) && (dt<0) && (max_improve<loss)
                break;
            end
            
        end
        
        if ((nb_col == 1)||(new_nb_col == 1))
            % Collision occurs and record the output
            final_col = 1;
            if (attack_start == 0)
                attack_start = st_tmp;
            end
            if (duration == 0)
                duration = t_tmp;
            end
            attack_id = att_id;
            victim_id = vim_id;
            deviation = dev_y;  
            % delete any zero rows
            out_mat = out_mat(any(out_mat,2),:);
            out_mat(row,:) = [final_col, init_fitness, fitness, attack_start, duration, attack_id, victim_id, deviation];
            break
        end
        if (attack_start == 0)
            attack_start = st_tmp;
        end
        if (duration == 0)
            duration = t_tmp;
        end
        attack_id = att_id;
        victim_id = vim_id;
        deviation = dev_y;  
        
        % delete any zero rows
        out_mat = out_mat(any(out_mat,2),:);
        out_mat(row,:) = [final_col, init_fitness, fitness, attack_start, duration, attack_id, victim_id, deviation];
        
    end
    
% --------- Write the outputs to the file -----------
%     out_mat = [final_col, init_fitness, fitness, attack_start, duration, x_fitness1, attack_id, victim_id, deviation]
    writematrix(out_mat, f_out, 'Delimiter', ',');

end
    
