# アルゴリズム詳細（日本語）

## 1. 6DoFモデル（クォータニオン姿勢）
状態量は `r, v, q, w, m` とし、姿勢はクォータニオン `q` を採用してオイラー角特異点を回避する。
回転運動学は `qdot = 0.5 * q * [0, wx, wy, wz]` を使用。

## 2. HIMES空力テーブル（Mach/alpha/beta）
本実装では簡易テーブル相当値を `coeffModel()` で生成し、
- Mach
- 迎角 alpha
- 横滑り角 beta
の3軸で8頂点を用いた三線形補間を行い `CL, CD, CY` を得る。

## 3. 制約の明示的導入
適応度に以下の制約ペナルティを直接加算する：
- 動圧 `q > 50 kPa`
- 熱流束 `q_dot > 1.2 MW/m^2`（簡易Sutton-Graves型近似）
- 荷重 `n > 6 g`

## 4. GA + リアルタイム化
- 選択: トーナメント
- 交叉: 一様交叉
- 突然変異: ガウス摂動
- エリート保存: 上位個体を次世代へコピー
- OpenMPで個体評価を並列化
- 経過時間が上限（例: 4秒）を超えたら世代ループを打ち切り

## 5. 出力
`results/best_trajectory.csv` に時系列で以下を記録：
高度、速度、飛行経路角、質量、制御入力、動圧、熱流束、荷重。

## 6. 参考文献
1. Deb, K., *Optimization for Engineering Design*, Prentice Hall, 2012.
2. Goldberg, D. E., *Genetic Algorithms in Search, Optimization and Machine Learning*, Addison-Wesley, 1989.
3. Stevens, B. L. et al., *Aircraft Control and Simulation*, 3rd ed., Wiley, 2015.
4. Anderson, J. D., *Hypersonic and High Temperature Gas Dynamics*, 2nd ed., AIAA, 2006.
5. Sutton, K., Graves, R. A., "A general stagnation-point convective-heating equation for arbitrary gas mixtures," NASA TR R-376, 1971.
