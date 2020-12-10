clear
clc
close all

% Make sure paths.csv and all relevant bag files are in the
% same directory as this script when you run it

bags = dir('*.bag'); % this calls/lists all bag files in directory
training_data = zeros(length(bags), 6);
empty_runs = [];
thresholds = [1 1.5 2 2.5 3 3.5 4];
filename = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\paths.csv";
paths = csvread(filename);

for j = 1:length(thresholds)
    for i = 1:length(bags)
        %% Collect data from run number
        name = convertStringsToChars(bags(i).name);
        r = strfind(name,'a');
        index = r(length(r)-1);
        %n = extractBetween(name,index+1,length(name)-4);
        %run = str2num(n{1})
        name_starting_from_run = name(strfind(name,'run'):end);
        split_name = split(name_starting_from_run, '_');
        run_string = split_name(1);
        run = split(run_string, 'run');
        run = run(2);
        run = str2double(run)

        name_starting_from_angle = name(strfind(name,'angle'):end);
        split_name = split(name_starting_from_angle, '.');
        angle_string = split_name(1);
        a = split(angle_string, 'angle');
        a = a(2);
        angle = str2double(a)

        i
        % calculate vel
        v_num = mod(run, 10);
        if v_num == 1
            vel = 0.2;
        elseif v_num == 2
            vel = 0.4;
        elseif v_num == 3
            vel = 0.6;
        elseif v_num == 4
            vel = 0.8;
        elseif v_num == 5
            vel = 1.0;
        elseif v_num == 6
            vel = 1.2;
        elseif v_num == 7
            vel = 1.4;
        elseif v_num == 8
            vel = 1.6;
        elseif v_num == 9
            vel = 1.8;
        elseif v_num == 0
            vel = 2.0;
        end

    % calculate goals from angle
        if angle == 0
            goal_x = 20;
            goal_y = 0;
            path = paths(:,1:2);
        elseif angle == 15
            goal_x = 19.659258260000000;
            goal_y = 2.588190451000000;
            path = paths(:,3:4);
        elseif angle == 30
            goal_x = 18.660254040000000;
            goal_y = 5;
            path = paths(:,5:6);
        elseif angle == 45
            goal_x = 17.071067810000000;
            goal_y = 7.071067812000000;
            path = paths(:,7:8);
        elseif angle == 60
            goal_x = 15;
            goal_y = 8.660254038000000;
            path = paths(:,9:10);
        elseif angle == 75
            goal_x = 12.588190450000000;
            goal_y = 9.659258263000000;
            path = paths(:,11:12);
        elseif angle == 90
            goal_x = 10;
            goal_y = 10;
            path = paths(:,13:14);
        elseif angle == 105
            goal_x = 7.411809549000000;
            goal_y = 9.659258263000000;
            path = paths(:,15:16);
        elseif angle == 120
            goal_x = 5;
            goal_y = 8.660254038000000;
            path = paths(:,17:18);
        elseif angle == 135
            goal_x = 2.928932188000000;
            goal_y = 7.071067812000000;
            path = paths(:,19:20);
        elseif angle == 150
            goal_x = 1.339745962000000;
            goal_y = 5;
            path = paths(:,21:22);
        elseif angle == 165
            goal_x = 0.340741737100000;
            goal_y = 2.588190451000000;
            path = paths(:,23:24);
        end

       % calculate mu
        mu = 0;
        if run <= 600
            mu = 0.009;
        elseif (600 < run) && (run <= 1200)
            mu = 0.09;
        elseif (1200 < run) && (run <= 1800)
            mu = 1;
        elseif (1800 < run) && (run <= 2400)
            mu = 0.05;
        elseif 2400 < run
            mu = 0.5;
        end


        %% Collect data from bag file
        topic1 = {'/gazebo/model_states'};
        bag = rosbag(bags(i).name);
        bSel1 = select(bag,'Topic', topic1);
        msgStructs1 = readMessages(bSel1,'DataFormat','struct');
        threshold = thresholds(j);
        try
            % Get index of robot position
            n1 = cellfun(@(m) (m.Name(1)),msgStructs1)';
            n2 = cellfun(@(m) (m.Name(2)),msgStructs1)';
            names = [n1(1), n2(1)];
            isRobot = cellfun(@(x)isequal(x,'jackal'),names);
            [row,index] = find(isRobot);

            % Extract Data
            x = cellfun(@(m) (m.Pose(index).Position.X),msgStructs1)';
            y = cellfun(@(m) (m.Pose(index).Position.Y),msgStructs1)';
            x_final = x(1, length(x));
            y_final = y(1, length(y));
            
            % calculate success
            min_d_error = zeros(length(x));
            for k = 1:length(x)
                for p = 1:length(path)
                    d_error = zeros(length(path));
                    d_error(p) = sqrt((path(p,1) - x(k)).^2 + (path(p,2) - y(k)).^2);
                end
                min_d_error(k) = min(d_error);
            end
            max_dist = max(min_d_error);
            
            if max_dist < threshold
                dist = sqrt((goal_x - x_final).^2 + (goal_y - y_final).^2);
                tolerance = 1;
                if dist <= tolerance
                    success = 1;
                end
            else
                success = 0;
            end

            row = [run, mu, vel, angle, threshold, success];
            training_data(i,:) = row;
        catch
            fprintf('run %i failed\n', run)
            row = [run, -1, -1, -1, -1, -1];
            training_data(i,:) = row;
            empty_runs = [empty_runs; run];
        end
    end
end

csvwrite('training_data.csv', training_data)
csvwrite('empty_runs.csv', empty_runs)
done = "done"
