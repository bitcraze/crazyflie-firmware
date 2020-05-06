clc
clear 
close all

%% Parameters 
Ts = 1/500;
Fs = 1/Ts;

T = 0.0617;                      % Time constant T [s]
alpha = 1 - exp(-Ts/T);   

filename = "data_1901.csv";

%% Import data

[tick, gyro_roll, gyro_pitch, gyro_yaw, cmd_thrust, cmd_roll,... 
 cmd_pitch, cmd_yaw, accelz] = import_logdata(filename);

time = (tick - tick(1)) / 1000;

%% Actuator dynamics
cmd_roll_a = filter([alpha], [1, -(1-alpha)], cmd_roll);
cmd_pitch_a = filter([alpha], [1, -(1-alpha)], cmd_pitch);
cmd_yaw_a = filter([alpha], [1, -(1-alpha)], cmd_yaw);
cmd_thrust_a = filter([alpha], [1, -(1-alpha)], cmd_thrust);


%% Data preprocessing

[b, a] = butter(4, 4/(Fs/2));       % compute filter coeffs

% Filtering gyro data to remove noise
gyro_roll_f = filter(b, a, gyro_roll);
gyro_pitch_f = filter(b, a, gyro_pitch);
gyro_yaw_f = filter(b, a, gyro_yaw);

% Filtering cmd data to remove noise
cmd_roll_f = filter(b, a, cmd_roll_a);
cmd_pitch_f = filter(b, a, cmd_pitch_a);
cmd_yaw_f = filter(b, a, cmd_yaw_a);

% Differtiating angular velocities to get angular accelerations
ang_accel_roll = [0; diff(gyro_roll_f)]/Ts;
ang_accel_pitch = [0; diff(gyro_pitch_f)]/Ts;
ang_accel_yaw = [0; diff(gyro_yaw_f)]/Ts;

% Differentiating 2nd time (for parameter estimation)
ang_accel_roll_d = [0; diff(ang_accel_roll)]/Ts;
ang_accel_pitch_d = [0; diff(ang_accel_pitch)]/Ts;
ang_accel_yaw_d = [0; diff(ang_accel_yaw)]/Ts;

% Differentiating cmd (for parameter estimation)
cmd_roll_fd = [0; diff(cmd_roll_f)]/Ts;
cmd_pitch_fd = [0; diff(cmd_pitch_f)]/Ts;
cmd_yaw_fd = [0; diff(cmd_yaw_f)]/Ts;
cmd_yaw_fdd = [0; diff(cmd_yaw_fd)]/Ts;


%% Compute control effectiveness parameters (least squares fitting)
G_roll = cmd_roll_fd \ ang_accel_roll_d;
G_pitch = cmd_pitch_fd \ ang_accel_pitch_d;
G_yaw = [cmd_yaw_fd cmd_yaw_fdd] \ ang_accel_yaw_d;


%%  Plot results 
display(['T = ' num2str(T) 's; Fs = ' num2str(Fs) 'Hz; alpha = ' num2str(alpha)])
display(['G1_roll = ' num2str(G_roll) '; G1_pitch = ' num2str(G_pitch) ...
         '; G1_yaw = ' num2str(G_yaw(1)) '; G2_yaw = ' num2str(G_yaw(2))])

% % Plot contributions of G matrices
% figure('Name','G1_yaw contribution');
% plot(ang_accel_yaw_d); hold on
% plot([cmd_yaw_fd 0*cmd_yaw_fdd]*G_yaw)
% xlim([0 inf]); grid on;
% legend('ang\_accel\_yaw\_d', 'cmd*G1')
% 
% figure('Name','G2_yaw contribution');
% plot(ang_accel_yaw_d); hold on
% plot([0*cmd_yaw_fd cmd_yaw_fdd]*G_yaw)
% xlim([0 inf]); grid on;
% legend('ang\_accel\_yaw\_d', 'cmdd*G2')