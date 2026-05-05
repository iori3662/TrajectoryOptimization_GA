# TrajectoryOptimization_GA

HIMES想定機に対する **6自由度軌道最適化（GA）** のC++実装です。

## 概要
- 姿勢はクォータニオンで表現（特異点回避）
- HIMES簡易空力テーブル（Mach/α/β）を3次元線形補間
- 制約（動圧・熱流束・荷重）を適応度へ明示的にペナルティ化
- OpenMP並列評価 + 計算時間上限による世代打ち切り
- 最良個体の時系列を `results/best_trajectory.csv` に保存

## 構成
- `src/main.cpp`: 6DoF+GA本体
- `scripts/visualize_results.m`: MATLAB可視化
- `docs/algorithm_details.md`: 手法詳細・文献
- `results/`: 出力先（`.gitignore`で除外）

## ビルドと実行
```bash
cmake -S . -B build
cmake --build build -j
./build/trajectory_ga
```

## MATLAB可視化
```matlab
cd('TrajectoryOptimization_GA')
run('scripts/visualize_results.m')
```


## JAXA文献反映について
本版ではJAXA公開ページ（R20EA2121, HIMICO Mach5の図情報）を参照し、空力テーブルの軸範囲（Mach/α/β）を文献準拠へ調整しています。数値生データは公開ページに含まれないため、現状は近似テーブル実装です。詳細は `docs/jaxa_himes_data_note.md` を参照してください。
