# TrajectoryOptimization_GA

HIMES（極超音速スペースプレーン想定）を対象に、**6自由度シミュレーション + 遺伝的アルゴリズム（GA）**で軌道最適化する C++ プロジェクトです。

## 概要
本プログラムはオンボード計算機でのリアルタイム誘導を想定し、以下の流れで動作します。
1. 6DoFダイナミクスで時系列制御入力に対する軌道を高速に評価。
2. GAで制御入力列（推力・迎角・バンク・ヨー）を反復改善。
3. 最良解を `results/best_trajectory.csv` に保存。
4. MATLABで可視化して誘導性能を確認。

## ファイル構成
- `src/main.cpp`: 6DoFシミュレータ + GA最適化本体
- `scripts/visualize_results.m`: MATLAB可視化スクリプト
- `docs/algorithm_details.md`: アルゴリズム詳細・文献
- `results/`: 実行結果出力先（`.gitignore` 対象）

## ビルド
```bash
cmake -S . -B build
cmake --build build -j
```

## 実行
```bash
./build/trajectory_ga
```

## MATLAB可視化
```matlab
cd('TrajectoryOptimization_GA')
run('scripts/visualize_results.m')
```

## 補足
本実装は「リアルタイム誘導向けの雛形」です。実機適用には、
- HIMES固有の空力テーブル（Mach/Re/α/β依存）
- 熱・荷重・動圧などの制約実装
- クォータニオン姿勢運動
- GA並列化と計算時間制約
の導入を推奨します。
