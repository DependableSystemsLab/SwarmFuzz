function cal_summary(csv1, csv2, csv3)

        col_mat = readmatrix(csv1);
        dist_mat = readmatrix(csv2);
        col_flag = 0;
        start_t = col_mat(1, 3);
        dur = col_mat(1, 4);
        % collision? if so, time
        col_idx = find(col_mat(:, 2)==1);
        if col_idx
            col_flag = 1;
            col_time = col_mat(min(col_idx),1);
            id_dist = find(dist_mat(:, 1) == col_time);
            dist_obs_min = dist_mat(id_dist, 2);
        else
            % minimal VDO
            dist_min_idx = find((dist_mat(:, 2))==(min(dist_mat(:, 2))));
            col_time = dist_mat(dist_min_idx, 1);
            dist_obs_min = dist_mat(dist_min_idx, 2)
        end
        sum_mat = [start_t, dur, col_flag, col_time, dist_obs_min];
        writematrix(sum_mat, csv3, 'Delimiter', ',');
        delete(csv1);
        delete(csv2);
end

