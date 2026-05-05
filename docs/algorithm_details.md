# アルゴリズム詳細（6DoF + GA）

## 1. 状態と運動方程式
本実装は以下の状態を持つ：
- 位置 `r` [m]（慣性座標）
- 速度 `v` [m/s]（慣性座標）
- 姿勢オイラー角 `eul=[roll,pitch,yaw]` [rad]
- 角速度 `w` [rad/s]
- 質量 `m` [kg]

運動は以下を考慮：
- 地球重力（中心力）
- 指数大気密度モデル
- 空力（揚力・抗力の簡易モデル）
- 推力
- 簡易姿勢追従（比例微分型のトルク生成）

時間積分は RK4 を使用。

## 2. 最適化問題
制御入力系列（各ノットで `thrustFrac, alpha, bank, yawCmd`）を最適化する。

目的関数は終端状態の目標との差と燃料消費の重み付き和：
- 高度誤差
- 速度誤差
- 飛行経路角誤差
- 燃料消費ペナルティ
- 地表下侵入に対する大ペナルティ

## 3. 遺伝的アルゴリズム
- 染色体：時系列制御入力列
- 初期集団：ランダム生成
- 選択：トーナメント選択
- 交叉：遺伝子ごとの一様交叉
- 突然変異：確率的ガウス摂動
- エリート保存：上位個体を次世代に直接コピー

## 4. MATLAB 可視化
C++実行後に `results/best_trajectory.csv` が出力される。
MATLABスクリプト `scripts/visualize_results.m` で、
- 高度・速度・飛行経路角・質量
- 推力・迎角・バンク・ヨー指令
を可視化できる。

## 5. 参考文献
1. Deb, K. *Optimization for Engineering Design: Algorithms and Examples*. Prentice Hall, 2012.
2. Goldberg, D. E. *Genetic Algorithms in Search, Optimization and Machine Learning*. Addison-Wesley, 1989.
3. Betts, J. T. *Practical Methods for Optimal Control and Estimation Using Nonlinear Programming*, 2nd ed., SIAM, 2010.
4. Stevens, B. L., Lewis, F. L., and Johnson, E. N. *Aircraft Control and Simulation*, 3rd ed., Wiley, 2015.
5. Anderson, J. D. *Hypersonic and High Temperature Gas Dynamics*, 2nd ed., AIAA, 2006.

> 注: 本コードの空力モデルはオンボード実装の雛形を目的とした簡易モデルであり、実運用には機体固有の高精度データ同定が必要。
