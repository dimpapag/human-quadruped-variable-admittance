clear all 
close all
warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');
% MATLAB Script to Load and Analyze Experiment Data

% Define the file names
file_names = {...
    'real_adm_csv_2024-08-23_17-10-10_fvEqual1.csv', ...
    'real_adm_csv_2024-08-23_17-20-21_fv_EQUAL0.csv', ...
    'real_adm_csv_2024-08-27_10-51-01_DLOW.csv', ...
    'real_adm_csv_2024-08-23_18-16-30_dHIGH.csv', ...
    'real_adm_csv_2024-08-23_18-19-35-dZED.csv'};

% Initialize a structure to hold data from each experiment
experiment_data = struct();

% Define the column names (adjust this if the order of columns changes)
column_names = {'time_sec', 'v_x', 'v_y', 'omega_z', 'f_x', 'f_y', 'tauZ', ...
                'p_x', 'p_y', 'theta_z', 'Power', 'd_factor', 'f_1', ...
                'f_2', 'f_3', 'f_4', 'f_m', 'f_v'};



% Loop over each file to load data
for i = 1:length(file_names)
    % Read the CSV file
    data = readtable(file_names{i});
    
    % Check if the last column does not have a header
    if width(data) == length(column_names) - 1
        data.Properties.VariableNames = column_names(1:end-1);
        data.f_v = data{:,end}; % Add the last column as 'f_v'
    else
        data.Properties.VariableNames = column_names;
    end
    
    % Store the data in the experiment_data structure
    experiment_data(i).file_name = file_names{i};
    experiment_data(i).data = data;
end

% FIGURE 1 _______________________________________________________
% Plotting the path (p_x vs. p_y) for the first two experiments
figure;
hold on;

% Plot path for the first experiment (with potential enabled) in blue
plot(experiment_data(1).data.p_x, experiment_data(1).data.p_y, 'b', 'LineWidth', 1.5);

% Plot path for the second experiment (without potential) in red
plot(experiment_data(2).data.p_x, experiment_data(2).data.p_y, 'r', 'LineWidth', 1.5);

% Add title, labels, and legend
title('Path (p_x vs. p_y)');
xlabel('p_x (m)');
ylabel('p_y (m)');
legend('with potential enabled', 'with potential disabled');

hold off;

% FIGURE 2 _______________________________________________________
% Creating a 6x1 subplot for f_1, f_2, f_3, f_4, f_m, and -f_v

figure;
T = max(experiment_data(1).data.time_sec);

% Subplot 1: f_1 with respect to time for the first two experiments
subplot(6,1,1); % 6 rows, 1 column, 1st plot
hold on;
plot(experiment_data(1).data.time_sec, experiment_data(1).data.f_1, 'b', 'LineWidth', 1.5);
plot(experiment_data(2).data.time_sec, experiment_data(2).data.f_1, 'r', 'LineWidth', 1.5);
plot([0 T], [-48 -48],'k--')
xlim([0,T])
ylim([-55,80])
% title('f_1 with respect to time');
% xlabel('Time (sec)', 'Interpreter','latex','FontSize', 14);
ylabel('$f_1$ (N)', 'Interpreter','latex','FontSize', 14);
legend('BAP enabled', 'BAP disabled', 'Interpreter','latex','FontSize', 14);
% grid on;
box on;
hold off;

% Subplot 2: f_2 with respect to time for the first two experiments
subplot(6,1,2); % 6 rows, 1 column, 2nd plot
hold on;
plot(experiment_data(1).data.time_sec, experiment_data(1).data.f_2, 'b', 'LineWidth', 1.5);
plot(experiment_data(2).data.time_sec, experiment_data(2).data.f_2, 'r', 'LineWidth', 1.5);
plot([0 T], [-48 -48],'k--')
xlim([0,T])
ylim([-55,80])
% title('f_2 with respect to time');
% xlabel('Time (sec)');
ylabel('$f_2$ (N)', 'Interpreter','latex','FontSize', 14);
% legend('with potential enabled', 'with potential disabled');
% grid on;
box on;
hold off;

% Subplot 3: f_3 with respect to time for the first two experiments
subplot(6,1,3); % 6 rows, 1 column, 3rd plot
hold on;
plot(experiment_data(1).data.time_sec, experiment_data(1).data.f_3, 'b', 'LineWidth', 1.5);
plot(experiment_data(2).data.time_sec, experiment_data(2).data.f_3, 'r', 'LineWidth', 1.5);
plot([0 T], [-48 -48],'k--')
xlim([0,T])
ylim([-55,80])
% title('f_3 with respect to time');
% xlabel('Time (sec)');
ylabel('$f_3$ (N)', 'Interpreter','latex','FontSize', 14);
% legend('with potential enabled', 'with potential disabled');
% grid on;
box on;
hold off;

% Subplot 4: f_4 with respect to time for the first two experiments
subplot(6,1,4); % 6 rows, 1 column, 4th plot
hold on;
plot(experiment_data(1).data.time_sec, experiment_data(1).data.f_4, 'b', 'LineWidth', 1.5);
plot(experiment_data(2).data.time_sec, experiment_data(2).data.f_4, 'r', 'LineWidth', 1.5);
plot([0 T], [-48 -48],'k--')
xlim([0,T])
ylim([-55,80])
% title('f_4 with respect to time');
% xlabel('Time (sec)');
ylabel('$f_4$ (N)', 'Interpreter','latex','FontSize', 14);
% legend('with potential enabled', 'with potential disabled');
% grid on;
box on;
hold off;

% Subplot 5: f_m with respect to time for the first two experiments
subplot(6,1,5); % 6 rows, 1 column, 5th plot
hold on;
plot(experiment_data(1).data.time_sec, experiment_data(1).data.f_m, 'b', 'LineWidth', 1.5);
plot(experiment_data(2).data.time_sec, experiment_data(2).data.f_m, 'r', 'LineWidth', 1.5);
plot([0 T], [0 0],'k--')
xlim([0,T])
ylim([-5,60])
% title('f_m with respect to time');
% xlabel('Time (sec)');
ylabel('$f_m$ (N)', 'Interpreter','latex','FontSize', 14);
% legend('with potential enabled', 'with potential disabled');
% grid on;
box on;
hold off;

% Subplot 6: -f_v with respect to time for the first experiment only
subplot(6,1,6); % 6 rows, 1 column, 6th plot
hold on;
plot(experiment_data(1).data.time_sec, -experiment_data(1).data.f_v, 'b', 'LineWidth', 1.5);
% title('-f_v with respect to time (with potential enabled)');
xlabel('Time (sec)', 'Interpreter','latex','FontSize', 14);
ylabel('-$f_v$ (N)', 'Interpreter','latex','FontSize', 14);
xlim([0,T])
% grid on;
box on;
hold off;

% FIGURE 3 _______________________________________________________
% Path plot (p_x vs. p_y) for experiments d_LOW, d_HIGH, and d_ZED
figure;
hold on;

% Plot path for d_LOW experiment in blue
plot(experiment_data(3).data.p_x, experiment_data(3).data.p_y, 'b', 'LineWidth', 1.5);

% Plot path for d_HIGH experiment in red
plot(experiment_data(4).data.p_x, experiment_data(4).data.p_y, 'r', 'LineWidth', 1.5);

% Plot path for d_ZED experiment in black
plot(experiment_data(5).data.p_x, experiment_data(5).data.p_y, 'k', 'LineWidth', 1.5);

% Add title, labels, and legend
title('Path (p_x vs. p_y)');
xlabel('p_x (m)');
ylabel('p_y (m)');
legend('d_LOW', 'd_HIGH', 'd_ZED');

hold off;


% FIGURE 4 _______________________________________________________
% Plotting p_x with respect to time for experiments d_LOW, d_HIGH, and d_ZED
figure;
hold on;

% Plot p_x with respect to time for d_LOW experiment in blue
plot(experiment_data(3).data.time_sec, experiment_data(3).data.v_x, 'b', 'LineWidth', 1.5);

% Plot p_x with respect to time for d_HIGH experiment in red
plot(experiment_data(4).data.time_sec, experiment_data(4).data.v_x, 'r', 'LineWidth', 1.5);

% Plot p_x with respect to time for d_ZED experiment in black
plot(experiment_data(5).data.time_sec, experiment_data(5).data.v_x, 'k', 'LineWidth', 1.5);

% Add title, labels, and legend
% title('p_x with respect to time');
xlabel('Time (sec)', 'Interpreter','latex','FontSize', 14 );
ylabel('$\dot{p}_x$ (m)' , 'Interpreter','latex','FontSize', 14);
legend('$\zeta=0.4$', '$\zeta=1.5$', 'variable $\zeta$', 'Interpreter','latex','FontSize', 14);
grid on;
box on;
xlim([0, max(experiment_data(3).data.time_sec)])
hold off;

% FIGURE 5 _______________________________________________________
% Calculate cumulative energy and plot with respect to time for experiments d_LOW, d_HIGH, and d_ZED
figure;
hold on;

% Define the time step
dt = 0.002;

% Compute cumulative energy for d_LOW experiment
energy_low = cumtrapz(experiment_data(3).data.time_sec, experiment_data(3).data.Power);

% Compute cumulative energy for d_HIGH experiment
energy_high = cumtrapz(experiment_data(4).data.time_sec, experiment_data(4).data.Power);

% Compute cumulative energy for d_ZED experiment
energy_zed = cumtrapz(experiment_data(5).data.time_sec, experiment_data(5).data.Power);

% Plot cumulative energy with respect to time for d_LOW experiment in blue
plot(experiment_data(3).data.time_sec, energy_low, 'b', 'LineWidth', 1.5);

% Plot cumulative energy with respect to time for d_HIGH experiment in red
plot(experiment_data(4).data.time_sec, energy_high, 'r', 'LineWidth', 1.5);

% Plot cumulative energy with respect to time for d_ZED experiment in black
plot(experiment_data(5).data.time_sec, energy_zed, 'k', 'LineWidth', 1.5);

% Add title, labels, and legend
% title('Cumulative Energy with respect to time');
xlabel('Time (sec)', 'Interpreter','latex','FontSize', 14);
ylabel('$E$ (J)', 'Interpreter','latex','FontSize', 14);
legend('$\zeta=0.4$', '$\zeta=1.5$', 'variable $\zeta$', 'Interpreter','latex','FontSize', 14);
box on ;
grid on;
xlim([0, max(experiment_data(3).data.time_sec)])
hold off;




