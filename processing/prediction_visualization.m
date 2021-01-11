%% Download Data
filenamep = 'predictions.csv';
filenamet = 'testing.csv';
preds = csvread(filenamep,1,0);
train = csvread(filenamet,1,0);

%% 3D Plot
mu = preds(:,1);
angle = preds(:,2);
pred_vel = preds(:,4);
figure(1)
plot3(mu, angle, pred_vel)
xlabel("Mu")
ylabel("Angle")
zlabel("Predicted Velocity")
hold off

%% Surface Plot
mu = zeros(60, 1);
angle = 0:1:165;
pred_vel = preds(:,4);
i = 1;
k = 1;
vels = zeros(length(angle), length(mu));
while i <= (length(preds)-165)
    mu(k) = preds(i,1);
    j = i+165;
    vels(:,k) = pred_vel(i:j,1);
    i = i+166;
    k = k+1;
end
figure(2)
surf(mu, angle, vels)
xlabel("Mu")
ylabel("Angle")
zlabel("Predicted Velocity")
hold off

%% All Predicted Data
i = 1;
k = 1;
mus = strings(length(preds)/166, 1);
while i <= (length(preds)-165)
    j = i+165;
    plot(preds(i:j,2), preds(i:j,4))
    hold on
    m = convertCharsToStrings(num2str(preds(i,1)));
    mus(k)= strcat("mu ", m);
    k = k+1;
    i = i+166;
end
l = legend(mus);
l.NumColumns=2;
xlabel("Angle")
ylabel("Predicted Velocity")

%% Plot Predicted vs Trained
i = 1;
% mu = 0.009
m009_pred = [];
m009_pred_05 = []; % prediction with threshold 0.5
m009_pred_1 = []; % prediction with threshold 1
m009_pred_15 = []; % prediction with threshold 1.5
m009_pred_2 = []; % prediction with threshold 2
m009_pred_25 = []; % prediction with threshold 2.5
m009_pred_3 = []; % prediction with threshold 3
m009_pred_35 = []; % prediction with threshold 3.5
m009_pred_4 = []; % prediction with threshold 4
% mu = 0.09
m09_pred = [];
m09_pred_05 = []; % prediction with threshold 0.5
m09_pred_1 = []; % prediction with threshold 1
m09_pred_15 = []; % prediction with threshold 1.5
m09_pred_2 = []; % prediction with threshold 2
m09_pred_25 = []; % prediction with threshold 2.5
m09_pred_3 = []; % prediction with threshold 3
m09_pred_35 = []; % prediction with threshold 3.5
m09_pred_4 = []; % prediction with threshold 4
% mu = 0.05
m05_pred = [];
m05_pred_05 = []; % prediction with threshold 0.5
m05_pred_1 = []; % prediction with threshold 1
m05_pred_15 = []; % prediction with threshold 1.5
m05_pred_2 = []; % prediction with threshold 2
m05_pred_25 = []; % prediction with threshold 2.5
m05_pred_3 = []; % prediction with threshold 3
m05_pred_35 = []; % prediction with threshold 3.5
m05_pred_4 = []; % prediction with threshold 4
% mu = 0.5
m5_pred = [];
m5_pred_05 = []; % prediction with threshold 0.5
m5_pred_1 = []; % prediction with threshold 1
m5_pred_15 = []; % prediction with threshold 1.5
m5_pred_2 = []; % prediction with threshold 2
m5_pred_25 = []; % prediction with threshold 2.5
m5_pred_3 = []; % prediction with threshold 3
m5_pred_35 = []; % prediction with threshold 3.5
m5_pred_4 = []; % prediction with threshold 4
% mu = 1
m1_pred = [];
m1_pred_05 = []; % prediction with threshold 0.5
m1_pred_1 = []; % prediction with threshold 1
m1_pred_15 = []; % prediction with threshold 1.5
m1_pred_2 = []; % prediction with threshold 2
m1_pred_25 = []; % prediction with threshold 2.5
m1_pred_3 = []; % prediction with threshold 3
m1_pred_35 = []; % prediction with threshold 3.5
m1_pred_4 = []; % prediction with threshold 4

while i <= (length(preds))
    m = preds(i,1);
    if m == 0.009
        m009_pred = [m009_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 0.5
            m009_pred_05 = [m009_pred_05; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.0
            m009_pred_1 = [m009_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.5
            m009_pred_15 = [m009_pred_15; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m009_pred_2 = [m009_pred_2; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.5
            m009_pred_25 = [m009_pred_25; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.0
            m009_pred_3 = [m009_pred_3; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.5
            m009_pred_35 = [m009_pred_35; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 4.0
            m009_pred_4 = [m009_pred_4; preds(i,2), preds(i,4)]; % fix
        end
    elseif m == 0.09
        m09_pred = [m09_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 0.5
            m09_pred_05 = [m09_pred_05; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.0
            m09_pred_1 = [m09_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.5
            m09_pred_15 = [m09_pred_15; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m09_pred_2 = [m09_pred_2; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.5
            m09_pred_25 = [m09_pred_25; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.0
            m09_pred_3 = [m09_pred_3; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.5
            m09_pred_35 = [m09_pred_35; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 4.0
            m09_pred_4 = [m09_pred_4; preds(i,2), preds(i,4)]; % fix
        end
    elseif m == 0.05
        m05_pred = [m5_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 0.5
            m05_pred_05 = [m05_pred_05; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.0
            m05_pred_1 = [m05_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.5
            m05_pred_15 = [m05_pred_15; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m05_pred_2 = [m05_pred_2; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.5
            m05_pred_25 = [m05_pred_25; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.0
            m05_pred_3 = [m05_pred_3; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.5
            m05_pred_35 = [m05_pred_35; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 4.0
            m05_pred_4 = [m05_pred_4; preds(i,2), preds(i,4)]; % fix
        end
    elseif m == 0.5
        m5_pred = [m5_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 0.5
            m5_pred_05 = [m5_pred_05; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.0
            m5_pred_1 = [m5_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.5
            m5_pred_15 = [m5_pred_15; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m5_pred_2 = [m5_pred_2; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.5
            m5_pred_25 = [m5_pred_25; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.0
            m5_pred_3 = [m5_pred_3; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.5
            m5_pred_35 = [m5_pred_35; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 4.0
            m5_pred_4 = [m5_pred_4; preds(i,2), preds(i,4)]; % fix
        end
    elseif m == 1
        m1_pred = [m1_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 0.5
            m1_pred_05 = [m1_pred_05; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.0
            m1_pred_1 = [m1_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 1.5
            m1_pred_15 = [m1_pred_15; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m1_pred_2 = [m1_pred_2; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.5
            m1_pred_25 = [m1_pred_25; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.0
            m1_pred_3 = [m1_pred_3; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 3.5
            m1_pred_35 = [m1_pred_35; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 4.0
            m1_pred_4 = [m1_pred_4; preds(i,2), preds(i,4)]; % fix
        end
    end
    i = i + 1;
end

k = 1;
% mu = 0.009
m009_train = [];
m009_train_05 = []; % training with threshold 0.5
m009_train_1 = []; % training with threshold 1
m009_train_15 = []; % training with threshold 1.5
m009_train_2 = []; % training with threshold 2
m009_train_25 = []; % training with threshold 2.5
m009_train_3 = []; % training with threshold 3
m009_train_35 = []; % training with threshold 3.5
m009_train_4 = []; % training with threshold 4
% mu = 0.09
m09_train = [];
m09_train_05 = []; % training with threshold 0.5
m09_train_1 = []; % training with threshold 1
m09_train_15 = []; % training with threshold 1.5
m09_train_2 = []; % training with threshold 2
m09_train_25 = []; % training with threshold 2.5
m09_train_3 = []; % training with threshold 3
m09_train_35 = []; % training with threshold 3.5
m09_train_4 = []; % training with threshold 4
% mu = 0.05
m05_train = [];
m05_train_05 = []; % training with threshold 0.5
m05_train_1 = []; % training with threshold 1
m05_train_15 = []; % training with threshold 1.5
m05_train_2 = []; % training with threshold 2
m05_train_25 = []; % training with threshold 2.5
m05_train_3 = []; % training with threshold 3
m05_train_35 = []; % training with threshold 3.5
m05_train_4 = []; % training with threshold 4
% mu = 0.5
m5_train = [];
m5_train_05 = []; % training with threshold 0.5
m5_train_1 = []; % training with threshold 1
m5_train_15 = []; % training with threshold 1.5
m5_train_2 = []; % training with threshold 2
m5_train_25 = []; % training with threshold 2.5
m5_train_3 = []; % training with threshold 3
m5_train_35 = []; % training with threshold 3.5
m5_train_4 = []; % training with threshold 4
% mu = 1
m1_train = [];
m1_train_05 = []; % training with threshold 0.5
m1_train_1 = []; % training with threshold 1
m1_train_15 = []; % training with threshold 1.5
m1_train_2 = []; % training with threshold 2
m1_train_25 = []; % training with threshold 2.5
m1_train_3 = []; % training with threshold 3
m1_train_35 = []; % training with threshold 3.5
m1_train_4 = []; % training with threshold 4
while k <= (length(train))
    m = train(k,1);
    if m == 0.009
        m009_train = [m009_train; train(k,2) train(k,4)];
        if train(k,3) == 0.5
            m009_train_05 = [m009_train_05; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.0
            m009_train_1 = [m009_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.5
            m009_train_15 = [m009_train_15; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m009_train_2 = [m009_train_2; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.5
            m009_train_25 = [m009_train_25; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.0
            m009_train_3 = [m009_train_3; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.5
            m009_train_35 = [m009_train_35; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 4.0
            m009_train_4 = [m009_train_4; train(k,2), train(k,4)]; % fix
        end
    elseif m == 0.09
        m09_train = [m09_train; train(k,2) train(k,4)];
        if train(k,3) == 0.5
            m09_train_05 = [m09_train_05; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.0
            m09_train_1 = [m09_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.5
            m09_train_15 = [m09_train_15; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m09_train_2 = [m09_train_2; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.5
            m09_train_25 = [m09_train_25; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.0
            m09_train_3 = [m09_train_3; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.5
            m09_train_35 = [m09_train_35; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 4.0
            m09_train_4 = [m09_train_4; train(k,2), train(k,4)]; % fix
        end
    elseif m == 0.05
        m05_train = [m05_train; train(k,2) train(k,4)];
        if train(k,3) == 0.5
            m05_train_05 = [m05_train_05; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.0
            m05_train_1 = [m05_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.5
            m05_train_15 = [m05_train_15; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m05_train_2 = [m05_train_2; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.5
            m05_train_25 = [m05_train_25; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.0
            m05_train_3 = [m05_train_3; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.5
            m05_train_35 = [m05_train_35; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 4.0
            m05_train_4 = [m05_train_4; train(k,2), train(k,4)]; % fix
        end
    elseif m == 0.5
        m5_train = [m5_train; train(k,2) train(k,4)];
        if train(k,3) == 0.5
            m5_train_05 = [m5_train_05; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.0
            m5_train_1 = [m5_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.5
            m5_train_15 = [m5_train_15; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m5_train_2 = [m5_train_2; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.5
            m5_train_25 = [m5_train_25; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.0
            m5_train_3 = [m5_train_3; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.5
            m5_train_35 = [m5_train_35; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 4.0
            m5_train_4 = [m5_train_4; train(k,2), train(k,4)]; % fix
        end
    elseif m == 1
        if train(k,3) == 0.5
            m1_train_05 = [m1_train_05; train(k,2), train(k,4)]; % fix
        end
        m1_train = [m1_train; train(k,2) train(k,4)];
        if train(k,3) == 1.0
            m1_train_1 = [m1_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 1.5
            m1_train_15 = [m1_train_15; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m1_train_2 = [m1_train_2; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.5
            m1_train_25 = [m1_train_25; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.0
            m1_train_3 = [m1_train_3; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 3.5
            m1_train_35 = [m1_train_35; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 4.0
            m1_train_4 = [m1_train_4; train(k,2), train(k,4)]; % fix
        end
    end
    k = k+1;
end


% % Mu = 0.009
% % average over threshold
% [ud, ix, iy] = unique(m009_pred(:,1));
% m009_pred_avg = [ud, accumarray(iy, m009_pred(:,2),[],@mean)];
% [ud, ix, iy] = unique(m009_train(:,1));
% m009_train_avg = [ud, accumarray(iy, m009_train(:,2),[],@mean)];
% figure(3)
% plot(m009_pred_avg(:,1), m009_pred_avg(:,2))
% hold on
% plot(m009_train_avg(:,1), m009_train_avg(:,2))
% title("Mu = 0.009")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% 
% % Mu = 0.09
% % average over threshold
% [ud, ix, iy] = unique(m09_pred(:,1));
% m09_pred_avg = [ud, accumarray(iy, m09_pred(:,2),[],@mean)];
% [ud, ix, iy] = unique(m09_train(:,1));
% m09_train_avg = [ud, accumarray(iy, m09_train(:,2),[],@mean)];
% figure(4)
% plot(m09_pred_avg(:,1), m09_pred_avg(:,2))
% hold on
% plot(m09_train_avg(:,1), m09_train_avg(:,2))
% title("Mu = 0.09")
% xlabel("Angle")
% ylabel("Predicted Velocity")
% ylim([1 3])
% legend("Predicted", "Training Data")
% hold off
% 
% % Mu = 0.5
% % average over threshold
% [ud, ix, iy] = unique(m5_pred(:,1));
% m5_pred_avg = [ud, accumarray(iy, m5_pred(:,2),[],@mean)];
% [ud, ix, iy] = unique(m5_train(:,1));
% m5_train_avg = [ud, accumarray(iy, m5_train(:,2),[],@mean)];
% figure(5)
% plot(m5_pred_avg(:,1), m5_pred_avg(:,2))
% hold on
% plot(m5_train_avg(:,1), m5_train_avg(:,2))
% title("Mu = 0.5")
% xlabel("Angle")
% ylabel("Predicted Velocity")
% ylim([1 3])
% legend("Predicted", "Training Data")
% hold off
% 
% % Mu = 1
% % average over threshold
% [ud, ix, iy] = unique(m1_pred(:,1));
% m1_pred_avg = [ud, accumarray(iy, m1_pred(:,2),[],@mean)];
% [ud, ix, iy] = unique(m1_train(:,1));
% m1_train_avg = [ud, accumarray(iy, m1_train(:,2),[],@mean)];
% figure(6)
% plot(m1_pred_avg(:,1), m1_pred_avg(:,2))
% hold on
% plot(m1_train_avg(:,1), m1_train_avg(:,2))
% title("Mu = 1")
% xlabel("Angle")
% ylabel("Predicted Velocity")
% ylim([1 3])
% legend("Predicted", "Training Data")
% hold off

%% Plot with threshold - Mu = 0.009
% % Mu = 0.009, Threshold = 0.5
% % sort
% m009_train_05 = sortrows(m009_train_05);
% % average
% [ud, ix, iy] = unique(m009_train_05(:,1));
% m009_train_05_avg = [ud, accumarray(iy, m009_train_05(:,2),[],@mean)];
% m009_pred_05 = sortrows(m009_pred_05);

% Mu = 0.009, Threshold = 1
% sort
m009_train_1 = sortrows(m009_train_1);
% average
[ud, ix, iy] = unique(m009_train_1(:,1));
m009_train_1_avg = [ud, accumarray(iy, m009_train_1(:,2),[],@mean)];
m009_pred_1 = sortrows(m009_pred_1);

% Mu = 0.009, Threshold = 1.5
% sort
m009_train_15 = sortrows(m009_train_15);
% average
[ud, ix, iy] = unique(m009_train_15(:,1));
m009_train_15_avg = [ud, accumarray(iy, m009_train_15(:,2),[],@mean)];
m009_pred_15 = sortrows(m009_pred_15);

% Mu = 0.009, Threshold = 2
% sort
m009_train_2 = sortrows(m009_train_2);
% average
[ud, ix, iy] = unique(m009_train_2(:,1));
m009_train_2_avg = [ud, accumarray(iy, m009_train_2(:,2),[],@mean)];
m009_pred_2 = sortrows(m009_pred_2);

% Mu = 0.009, Threshold = 2.5
% sort
m009_train_25 = sortrows(m009_train_25);
% average
[ud, ix, iy] = unique(m009_train_25(:,1));
m009_train_25_avg = [ud, accumarray(iy, m009_train_25(:,2),[],@mean)];
m009_pred_25 = sortrows(m009_pred_25);

% Mu = 0.009, Threshold = 3
% sort
m009_train_3 = sortrows(m009_train_3);
% average
[ud, ix, iy] = unique(m009_train_3(:,1));
m009_train_3_avg = [ud, accumarray(iy, m009_train_3(:,2),[],@mean)];
m009_pred_3 = sortrows(m009_pred_3);

% % Mu = 0.009, Threshold = 3.5
% % sort
% m009_train_35 = sortrows(m009_train_35);
% % average
% [ud, ix, iy] = unique(m009_train_35(:,1));
% m009_train_35_avg = [ud, accumarray(iy, m009_train_35(:,2),[],@mean)];
% m009_pred_35 = sortrows(m009_pred_35);

% % Mu = 0.009, Threshold = 4
% % sort
% m009_train_4 = sortrows(m009_train_4);
% % average
% [ud, ix, iy] = unique(m009_train_4(:,1));
% m009_train_4_avg = [ud, accumarray(iy, m009_train_4(:,2),[],@mean)];
% m009_pred_4 = sortrows(m009_pred_4);

% Create plots
figure(3)
t = tiledlayout(3,2); % Requires R2019b or later
% nexttile
% plot(m009_pred_05(:,1), m009_pred_05(:,2))
% hold on
% plot(m009_train_05_avg(:,1), m009_train_05_avg(:,2))
% title("Mu = 0.009, threshold = 0.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
nexttile
plot(m009_pred_1(:,1), m009_pred_1(:,2))
hold on
plot(m009_train_1_avg(:,1), m009_train_1_avg(:,2))
title("Mu = 0.009, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m009_pred_15(:,1), m009_pred_15(:,2))
hold on
plot(m009_train_15_avg(:,1), m009_train_15_avg(:,2))
title("Mu = 0.009, threshold = 1.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m009_pred_2(:,1), m009_pred_2(:,2))
hold on
plot(m009_train_2_avg(:,1), m009_train_2_avg(:,2))
title("Mu = 0.009, threshold = 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m009_pred_25(:,1), m009_pred_25(:,2))
hold on
plot(m009_train_25_avg(:,1), m009_train_25_avg(:,2))
title("Mu = 0.009, threshold = 2.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m009_pred_3(:,1), m009_pred_3(:,2))
hold on
plot(m009_train_3_avg(:,1), m009_train_3_avg(:,2))
title("Mu = 0.009, threshold = 3 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
% nexttile
% plot(m009_pred_35(:,1), m009_pred_35(:,2))
% hold on
% plot(m009_train_35_avg(:,1), m009_train_35_avg(:,2))
% title("Mu = 0.009, threshold = 3.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';
% nexttile
% plot(m009_pred_4(:,1), m009_pred_4(:,2))
% hold on
% plot(m009_train_4_avg(:,1), m009_train_4_avg(:,2))
% title("Mu = 0.009, threshold = 4 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';


%% Plot with threshold - Mu = 0.09
% % Mu = 0.09, Threshold = 0.5
% % sort
% m09_train_05 = sortrows(m09_train_05);
% % average
% [ud, ix, iy] = unique(m09_train_05(:,1));
% m09_train_05_avg = [ud, accumarray(iy, m09_train_05(:,2),[],@mean)];
% m09_pred_05 = sortrows(m09_pred_05);

% Mu = 0.09, Threshold = 1
% sort
m09_train_1 = sortrows(m09_train_1);
% average
[ud, ix, iy] = unique(m09_train_1(:,1));
m09_train_1_avg = [ud, accumarray(iy, m09_train_1(:,2),[],@mean)];
m09_pred_1 = sortrows(m09_pred_1);

% Mu = 0.09, Threshold = 1.5
% sort
m09_train_15 = sortrows(m09_train_15);
% average
[ud, ix, iy] = unique(m09_train_15(:,1));
m09_train_15_avg = [ud, accumarray(iy, m09_train_15(:,2),[],@mean)];
m09_pred_15 = sortrows(m09_pred_15);

% Mu = 0.09, Threshold = 2
% sort
m09_train_2 = sortrows(m09_train_2);
% average
[ud, ix, iy] = unique(m09_train_2(:,1));
m09_train_2_avg = [ud, accumarray(iy, m09_train_2(:,2),[],@mean)];
m09_pred_2 = sortrows(m09_pred_2);

% Mu = 0.09, Threshold = 2.5
% sort
m09_train_25 = sortrows(m09_train_25);
% average
[ud, ix, iy] = unique(m09_train_25(:,1));
m09_train_25_avg = [ud, accumarray(iy, m09_train_25(:,2),[],@mean)];
m09_pred_25 = sortrows(m09_pred_25);

% Mu = 0.09, Threshold = 3
% sort
m09_train_3 = sortrows(m09_train_3);
% average
[ud, ix, iy] = unique(m09_train_3(:,1));
m09_train_3_avg = [ud, accumarray(iy, m09_train_3(:,2),[],@mean)];
m09_pred_3 = sortrows(m09_pred_3);

% % Mu = 0.09, Threshold = 3.5
% % sort
% m09_train_35 = sortrows(m09_train_35);
% % average
% [ud, ix, iy] = unique(m09_train_35(:,1));
% m09_train_35_avg = [ud, accumarray(iy, m09_train_35(:,2),[],@mean)];
% m09_pred_35 = sortrows(m09_pred_35);
% 
% % Mu = 0.09, Threshold = 4
% % sort
% m09_train_4 = sortrows(m09_train_4);
% % average
% [ud, ix, iy] = unique(m09_train_4(:,1));
% m09_train_4_avg = [ud, accumarray(iy, m09_train_4(:,2),[],@mean)];
% m09_pred_4 = sortrows(m09_pred_4);

% Create plots
figure(4)
t = tiledlayout(3,2); % Requires R2019b or later
% nexttile
% plot(m09_pred_05(:,1), m09_pred_05(:,2))
% hold on
% plot(m09_train_05_avg(:,1), m09_train_05_avg(:,2))
% title("Mu = 0.09, threshold = 0.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
nexttile
plot(m09_pred_1(:,1), m09_pred_1(:,2))
hold on
plot(m09_train_1_avg(:,1), m09_train_1_avg(:,2))
title("Mu = 0.09, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m09_pred_15(:,1), m09_pred_15(:,2))
hold on
plot(m09_train_15_avg(:,1), m09_train_15_avg(:,2))
title("Mu = 0.09, threshold = 1.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m09_pred_2(:,1), m09_pred_2(:,2))
hold on
plot(m09_train_2_avg(:,1), m09_train_2_avg(:,2))
title("Mu = 0.09, threshold = 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m09_pred_25(:,1), m09_pred_25(:,2))
hold on
plot(m09_train_25_avg(:,1), m09_train_25_avg(:,2))
title("Mu = 0.09, threshold = 2.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m09_pred_3(:,1), m09_pred_3(:,2))
hold on
plot(m09_train_3_avg(:,1), m09_train_3_avg(:,2))
title("Mu = 0.09, threshold = 3 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
% nexttile
% plot(m09_pred_35(:,1), m09_pred_35(:,2))
% hold on
% plot(m09_train_35_avg(:,1), m09_train_35_avg(:,2))
% title("Mu = 0.09, threshold = 3.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';
% nexttile
% plot(m09_pred_4(:,1), m09_pred_4(:,2))
% hold on
% plot(m09_train_4_avg(:,1), m09_train_4_avg(:,2))
% title("Mu = 0.09, threshold = 4 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';

%% Plot with threshold - Mu = 0.05
% Mu = 0.05, Threshold = 0.5
% % sort
% m05_train_05 = sortrows(m05_train_05);
% % average
% [ud, ix, iy] = unique(m05_train_05(:,1));
% m05_train_05_avg = [ud, accumarray(iy, m05_train_05(:,2),[],@mean)];
% m05_pred_05 = sortrows(m05_pred_05);

% Mu = 0.05, Threshold = 1
% sort
m05_train_1 = sortrows(m05_train_1);
% average
[ud, ix, iy] = unique(m05_train_1(:,1));
m05_train_1_avg = [ud, accumarray(iy, m05_train_1(:,2),[],@mean)];
m05_pred_1 = sortrows(m05_pred_1);

% Mu = 0.05, Threshold = 1.5
% sort
m05_train_15 = sortrows(m05_train_15);
% average
[ud, ix, iy] = unique(m05_train_15(:,1));
m05_train_15_avg = [ud, accumarray(iy, m05_train_15(:,2),[],@mean)];
m05_pred_15 = sortrows(m05_pred_15);

% Mu = 0.05, Threshold = 2
% sort
m05_train_2 = sortrows(m05_train_2);
% average
[ud, ix, iy] = unique(m05_train_2(:,1));
m05_train_2_avg = [ud, accumarray(iy, m05_train_2(:,2),[],@mean)];
m05_pred_2 = sortrows(m05_pred_2);

% Mu = 0.05, Threshold = 2.5
% sort
m05_train_25 = sortrows(m05_train_25);
% average
[ud, ix, iy] = unique(m05_train_25(:,1));
m05_train_25_avg = [ud, accumarray(iy, m05_train_25(:,2),[],@mean)];
m05_pred_25 = sortrows(m05_pred_25);

% Mu = 0.05, Threshold = 3
% sort
m05_train_3 = sortrows(m05_train_3);
% average
[ud, ix, iy] = unique(m05_train_3(:,1));
m05_train_3_avg = [ud, accumarray(iy, m05_train_3(:,2),[],@mean)];
m05_pred_3 = sortrows(m05_pred_3);

% % Mu = 0.05, Threshold = 3.5
% % sort
% m05_train_35 = sortrows(m05_train_35);
% % average
% [ud, ix, iy] = unique(m05_train_35(:,1));
% m05_train_35_avg = [ud, accumarray(iy, m05_train_35(:,2),[],@mean)];
% m05_pred_35 = sortrows(m05_pred_35);
% 
% % Mu = 0.05, Threshold = 4
% % sort
% m05_train_4 = sortrows(m05_train_4);
% % average
% [ud, ix, iy] = unique(m05_train_4(:,1));
% m05_train_4_avg = [ud, accumarray(iy, m05_train_4(:,2),[],@mean)];
% m05_pred_4 = sortrows(m05_pred_4);

% Create plots
figure(5)
t = tiledlayout(3,2); % Requires R2019b or later
% nexttile
% plot(m05_pred_05(:,1), m05_pred_05(:,2))
% hold on
% plot(m05_train_05_avg(:,1), m05_train_05_avg(:,2))
% title("Mu = 0.05, threshold = 0.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
nexttile
plot(m05_pred_1(:,1), m05_pred_1(:,2))
hold on
plot(m05_train_1_avg(:,1), m05_train_1_avg(:,2))
title("Mu = 0.05, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m05_pred_15(:,1), m05_pred_15(:,2))
hold on
plot(m05_train_15_avg(:,1), m05_train_15_avg(:,2))
title("Mu = 0.05, threshold = 1.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m05_pred_2(:,1), m05_pred_2(:,2))
hold on
plot(m05_train_2_avg(:,1), m05_train_2_avg(:,2))
title("Mu = 0.05, threshold = 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m05_pred_25(:,1), m05_pred_25(:,2))
hold on
plot(m05_train_25_avg(:,1), m05_train_25_avg(:,2))
title("Mu = 0.05, threshold = 2.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m05_pred_3(:,1), m05_pred_3(:,2))
hold on
plot(m05_train_3_avg(:,1), m05_train_3_avg(:,2))
title("Mu = 0.05, threshold = 3 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
% nexttile
% plot(m05_pred_35(:,1), m05_pred_35(:,2))
% hold on
% plot(m05_train_35_avg(:,1), m05_train_35_avg(:,2))
% title("Mu = 0.05, threshold = 3.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';
% nexttile
% plot(m05_pred_4(:,1), m05_pred_4(:,2))
% hold on
% plot(m05_train_4_avg(:,1), m05_train_4_avg(:,2))
% title("Mu = 0.05, threshold = 4 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';

%% Plot with threshold - Mu = 0.5
% % Mu = 0.5, Threshold = 0.5
% % sort
% m5_train_05 = sortrows(m5_train_05);
% % average
% [ud, ix, iy] = unique(m1_train_1(:,1));
% m5_train_05_avg = [ud, accumarray(iy, m5_train_05(:,2),[],@mean)];
% m5_pred_05 = sortrows(m5_pred_05);

% Mu = 0.5, Threshold = 1
% sort
m5_train_1 = sortrows(m5_train_1);
% average
[ud, ix, iy] = unique(m5_train_1(:,1));
m5_train_1_avg = [ud, accumarray(iy, m5_train_1(:,2),[],@mean)];
m5_pred_1 = sortrows(m5_pred_1);

% Mu = 0.5, Threshold = 1.5
% sort
m5_train_15 = sortrows(m5_train_15);
% average
[ud, ix, iy] = unique(m5_train_15(:,1));
m5_train_15_avg = [ud, accumarray(iy, m5_train_15(:,2),[],@mean)];
m5_pred_15 = sortrows(m5_pred_15);

% Mu = 0.5, Threshold = 2
% sort
m5_train_2 = sortrows(m5_train_2);
% average
[ud, ix, iy] = unique(m5_train_2(:,1));
m5_train_2_avg = [ud, accumarray(iy, m5_train_2(:,2),[],@mean)];
m5_pred_2 = sortrows(m5_pred_2);

% Mu = 0.5, Threshold = 2.5
% sort
m5_train_25 = sortrows(m5_train_25);
% average
[ud, ix, iy] = unique(m5_train_25(:,1));
m5_train_25_avg = [ud, accumarray(iy, m5_train_25(:,2),[],@mean)];
m5_pred_25 = sortrows(m5_pred_25);

% Mu = 0.5, Threshold = 3
% sort
m5_train_3 = sortrows(m5_train_3);
% average
[ud, ix, iy] = unique(m5_train_3(:,1));
m5_train_3_avg = [ud, accumarray(iy, m5_train_3(:,2),[],@mean)];
m5_pred_3 = sortrows(m5_pred_3);

% % Mu = 0.5, Threshold = 3.5
% % sort
% m5_train_35 = sortrows(m5_train_35);
% % average
% [ud, ix, iy] = unique(m5_train_35(:,1));
% m5_train_35_avg = [ud, accumarray(iy, m5_train_35(:,2),[],@mean)];
% m5_pred_35 = sortrows(m5_pred_35);
% 
% % Mu = 0.5, Threshold = 4
% % sort
% m5_train_4 = sortrows(m5_train_4);
% % average
% [ud, ix, iy] = unique(m5_train_4(:,1));
% m5_train_4_avg = [ud, accumarray(iy, m5_train_4(:,2),[],@mean)];
% m5_pred_4 = sortrows(m5_pred_4);

% Create plots
figure(6)
t = tiledlayout(3,2); % Requires R2019b or later
% nexttile
% plot(m5_pred_05(:,1), m5_pred_05(:,2))
% hold on
% plot(m5_train_05_avg(:,1), m5_train_05_avg(:,2))
% title("Mu = 0.5, threshold = 0.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
nexttile
plot(m5_pred_1(:,1), m5_pred_1(:,2))
hold on
plot(m5_train_1_avg(:,1), m5_train_1_avg(:,2))
title("Mu = 0.5, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m5_pred_15(:,1), m5_pred_15(:,2))
hold on
plot(m5_train_15_avg(:,1), m5_train_15_avg(:,2))
title("Mu = 0.5, threshold = 1.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m5_pred_2(:,1), m5_pred_2(:,2))
hold on
plot(m5_train_2_avg(:,1), m5_train_2_avg(:,2))
title("Mu = 0.5, threshold = 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m5_pred_25(:,1), m5_pred_25(:,2))
hold on
plot(m5_train_25_avg(:,1), m5_train_25_avg(:,2))
title("Mu = 0.5, threshold = 2.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m5_pred_3(:,1), m5_pred_3(:,2))
hold on
plot(m5_train_3_avg(:,1), m5_train_3_avg(:,2))
title("Mu = 0.5, threshold = 3 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
% nexttile
% plot(m5_pred_35(:,1), m5_pred_35(:,2))
% hold on
% plot(m5_train_35_avg(:,1), m5_train_35_avg(:,2))
% title("Mu = 0.5, threshold = 3.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';
% nexttile
% plot(m5_pred_4(:,1), m5_pred_4(:,2))
% hold on
% plot(m5_train_4_avg(:,1), m5_train_4_avg(:,2))
% title("Mu = 0.5, threshold = 4 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';


%% Plot with threshold - Mu = 1
% % Mu = 1, Threshold = 0.5
% % sort
% m1_train_05 = sortrows(m1_train_05);
% % average
% [ud, ix, iy] = unique(m1_train_05(:,1));
% m1_train_05_avg = [ud, accumarray(iy, m1_train_05(:,2),[],@mean)];
% m1_pred_05 = sortrows(m1_pred_05);

% Mu = 1, Threshold = 1
% sort
m1_train_1 = sortrows(m1_train_1);
% average
[ud, ix, iy] = unique(m1_train_1(:,1));
m1_train_1_avg = [ud, accumarray(iy, m1_train_1(:,2),[],@mean)];
m1_pred_1 = sortrows(m1_pred_1);

% Mu = 1, Threshold = 1.5
% sort
m1_train_15 = sortrows(m1_train_15);
% average
[ud, ix, iy] = unique(m1_train_15(:,1));
m1_train_15_avg = [ud, accumarray(iy, m1_train_15(:,2),[],@mean)];
m1_pred_15 = sortrows(m1_pred_15);

% Mu = 1, Threshold = 2
% sort
m1_train_2 = sortrows(m1_train_2);
% average
[ud, ix, iy] = unique(m1_train_2(:,1));
m1_train_2_avg = [ud, accumarray(iy, m1_train_2(:,2),[],@mean)];
m1_pred_2 = sortrows(m1_pred_2);

% Mu = 01, Threshold = 2.5
% sort
m1_train_25 = sortrows(m1_train_25);
% average
[ud, ix, iy] = unique(m1_train_25(:,1));
m1_train_25_avg = [ud, accumarray(iy, m1_train_25(:,2),[],@mean)];
m1_pred_25 = sortrows(m1_pred_25);

% Mu = 1, Threshold = 3
% sort
m1_train_3 = sortrows(m1_train_3);
% average
[ud, ix, iy] = unique(m1_train_3(:,1));
m1_train_3_avg = [ud, accumarray(iy, m1_train_3(:,2),[],@mean)];
m1_pred_3 = sortrows(m1_pred_3);

% % Mu = 1, Threshold = 3.5
% % sort
% m1_train_35 = sortrows(m1_train_35);
% % average
% [ud, ix, iy] = unique(m1_train_35(:,1));
% m1_train_35_avg = [ud, accumarray(iy, m1_train_35(:,2),[],@mean)];
% m1_pred_35 = sortrows(m1_pred_35);
% 
% % Mu = 1, Threshold = 4
% % sort
% m1_train_4 = sortrows(m1_train_4);
% % average
% [ud, ix, iy] = unique(m1_train_4(:,1));
% m1_train_4_avg = [ud, accumarray(iy, m1_train_4(:,2),[],@mean)];
% m1_pred_4 = sortrows(m1_pred_4);

% Create plots
figure(7)
t = tiledlayout(3,2); % Requires R2019b or later
% nexttile
% plot(m1_pred_05(:,1), m1_pred_05(:,2))
% hold on
% plot(m1_train_05_avg(:,1), m1_train_05_avg(:,2))
% title("Mu = 1, threshold = 0.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
nexttile
plot(m1_pred_1(:,1), m1_pred_1(:,2))
hold on
plot(m1_train_1_avg(:,1), m1_train_1_avg(:,2))
title("Mu = 1, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m1_pred_15(:,1), m1_pred_15(:,2))
hold on
plot(m1_train_15_avg(:,1), m1_train_15_avg(:,2))
title("Mu = 1, threshold = 1.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
nexttile
plot(m1_pred_2(:,1), m1_pred_2(:,2))
hold on
plot(m1_train_2_avg(:,1), m1_train_2_avg(:,2))
title("Mu = 1, threshold = 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m1_pred_25(:,1), m1_pred_25(:,2))
hold on
plot(m1_train_25_avg(:,1), m1_train_25_avg(:,2))
title("Mu = 1, threshold = 2.5 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
nexttile
plot(m1_pred_3(:,1), m1_pred_3(:,2))
hold on
plot(m1_train_3_avg(:,1), m1_train_3_avg(:,2))
title("Mu = 1, threshold = 3 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off
t.Padding = 'none';
t.TileSpacing = 'none';
% nexttile
% plot(m1_pred_35(:,1), m1_pred_35(:,2))
% hold on
% plot(m1_train_35_avg(:,1), m1_train_35_avg(:,2))
% title("Mu = 1, threshold = 3.5 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';
% nexttile
% plot(m1_pred_4(:,1), m1_pred_4(:,2))
% hold on
% plot(m1_train_4_avg(:,1), m1_train_4_avg(:,2))
% title("Mu = 1, threshold = 4 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted", "Training Data")
% hold off
% t.Padding = 'none';
% t.TileSpacing = 'none';


%% Plot multiple thresholds
% Mu = 0.009, 2 thresholds
% sort
m009_train_2 = sortrows(m009_train_2);
% average
[ud, ix, iy] = unique(m009_train_2(:,1));
m009_train_2_avg = [ud, accumarray(iy, m009_train_2(:,2),[],@mean)];

% figure(28)
% plot(m009_pred_1(:,1), m009_pred_1(:,2))
% hold on
% plot(m009_train_1_avg(:,1), m009_train_1_avg(:,2))
% plot(m009_pred_2(:,1), m009_pred_2(:,2))
% plot(m009_train_2_avg(:,1), m009_train_2_avg(:,2))
% title("Mu = 0.009, thresholds = 1 and 2 m")
% xlabel("Angle")
% ylabel("Velocity")
% ylim([0 2])
% legend("Predicted (1 m)", "Training (1 m)", "Predicted (2 m)", "Training (2 m)")
% %legend("Predicted", "Training Data")
% hold off

%% Surface Plot 2
mu = zeros(60, 1);
thresholds = zeros(60, 1);
angle = 0:1:165;
threshold = preds(:,3);
pred_vel = preds(:,4);
i = 1;
k = 1;
vels = zeros(length(angle), length(mu));
thresh = zeros(length(angle), length(mu));
while i <= (length(preds)-165)
    mu(k) = preds(i,1);
    thresholds(k) = preds(i,3);
    j = i+165;
    thresh(:,k) = threshold(i:j,1);
    vels(:,k) = pred_vel(i:j,1);
    i = i+166;
    k = k+1;
end
% figure(8)
% surf(thresholds, angle, vels)
% xlabel("thresh")
% ylabel("Angle")
% zlabel("Predicted Velocity")
% % surf(threshold, angle, vels)
% % xlabel("Mu")
% % ylabel("Angle")
% % zlabel("Predicted Velocity")
% hold off

%% please work

[X,Y,Z] = sphere(16);
x = [0.5*X(:); 0.75*X(:); X(:)];
vel_x = preds(:,4);
y = [0.5*Y(:); 0.75*Y(:); Y(:)];
mu_y = preds(:,1);
z = [0.5*Z(:); 0.75*Z(:); Z(:)];
ang_z = preds(:,2);

S = ones(1,size(vel_x, 1)); 
C = preds(:,3);

figure(9)
scatter3(vel_x,mu_y,ang_z,S,C)
xlabel("Predicted Velocity")
ylabel("Mu")
zlabel("Angle")
colorbar
view(40,35)
