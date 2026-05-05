% TrajectoryOptimization_GA 可視化スクリプト（日本語版）
clear; clc;

result_file = fullfile('results', 'best_trajectory.csv');
if ~isfile(result_file)
    error('結果ファイルが見つかりません: %s\n先に trajectory_ga を実行してください。', result_file);
end

T = readtable(result_file);

figure('Name','軌道状態','Color','w');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(T.time_s, T.altitude_m/1000,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('高度 [km]');
nexttile; plot(T.time_s, T.speed_mps,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('速度 [m/s]');
nexttile; plot(T.time_s, T.gamma_deg,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('飛行経路角 [deg]');
nexttile; plot(T.time_s, T.mass_kg,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('質量 [kg]');
nexttile; plot(T.time_s, T.qdyn_pa/1000,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('動圧 [kPa]');
nexttile; plot(T.time_s, T.load_g,'LineWidth',1.4); grid on; xlabel('時間 [s]'); ylabel('荷重 [g]');

figure('Name','制御入力と熱流束','Color','w');
tiledlayout(3,2,'Padding','compact','TileSpacing','compact');
nexttile; plot(T.time_s, T.thrust_frac,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('推力率 [-]');
nexttile; plot(T.time_s, T.alpha_deg,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('迎角 [deg]');
nexttile; plot(T.time_s, T.beta_deg,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('横滑り角 [deg]');
nexttile; plot(T.time_s, T.bank_deg,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('バンク角 [deg]');
nexttile; plot(T.time_s, T.yaw_deg,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('ヨー指令 [deg]');
nexttile; plot(T.time_s, T.heat_wm2/1e6,'LineWidth',1.2); grid on; xlabel('時間 [s]'); ylabel('熱流束 [MW/m^2]');

disp('可視化が完了しました。');
