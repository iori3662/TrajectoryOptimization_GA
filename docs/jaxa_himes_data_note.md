# JAXA文献に基づくHIMES空力データ反映メモ

## 参照文献
- JAXA Supercomputer System Annual Report 2020, Report Number: R20EA2121  
  "Numerical Analyses on Hypersonic Experimental Aircraft"（HIMICO）
  - Mach 5 条件で、AoA = 0 deg および -15 deg の流れ場
  - Mach 5 条件で、縦3分力の風洞試験比較図（Fig.4）

## 反映方法
公開ページでは数値表（CL/CD/Cm の生データ）が提供されていないため、
本実装では以下の方針で**JAXA文献準拠の近似テーブル**を導入した。
1. AoA 範囲を文献図に合わせて `[-15, 0, 15] deg` を基準化
2. Mach 範囲は極超音速周辺として `[3, 5, 7]` を採用
3. beta 範囲は小さな横滑りを想定して `[-6, 0, 6] deg` を採用
4. CL/CD/CY は文献の傾向（AoA増加でCL増、CD増、βでCY発生）を満たす形で近似
5. 補間は三線形補間

## 今後
- JAXA論文本文または風洞データ提供元から**数値テーブル**が取得でき次第、
  `coeffModel()` を廃止し、CSV読込型の空力DBへ置き換えること。
