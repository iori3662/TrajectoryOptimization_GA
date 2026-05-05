% MATLAB visualization for TrajectoryOptimization_GA outputs
clear; clc;

result_file = fullfile('results', 'best_trajectory.csv');
if ~isfile(result_file)
    error('Result file not found: %s\nRun trajectory_ga first.', result_file);
end

T = readtable(result_file);

figure('Name','Trajectory Overview','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(T.time_s, T.altitude_m/1000,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Altitude [km]'); grid on; title('Altitude');

nexttile;
plot(T.time_s, T.speed_mps,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Speed [m/s]'); grid on; title('Speed');

nexttile;
plot(T.time_s, T.gamma_deg,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('\gamma [deg]'); grid on; title('Flight-path Angle');

nexttile;
plot(T.time_s, T.mass_kg,'LineWidth',1.5);
xlabel('Time [s]'); ylabel('Mass [kg]'); grid on; title('Mass');

figure('Name','Control Profile','Color','w');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
plot(T.time_s, T.thrust_frac,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Thrust frac [-]'); grid on; title('Thrust');

nexttile;
plot(T.time_s, T.alpha_deg,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('\alpha [deg]'); grid on; title('Angle of attack');

nexttile;
plot(T.time_s, T.bank_deg,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Bank [deg]'); grid on; title('Bank');

nexttile;
plot(T.time_s, T.yaw_deg,'LineWidth',1.3);
xlabel('Time [s]'); ylabel('Yaw cmd [deg]'); grid on; title('Yaw command');

disp('Visualization complete.');
