# 対応と制限

対象はrapier-rope 0.1.0です。

| 項目 | 対応・境界 |
|---|---|
| Rust | MSRV 1.90、edition 2024 |
| Rapier | 3D、CPU、rapier3dまたはrapier3d-f64 0.36.0をre-export |
| 精度 | default f32。f64はdefaultを切って選択。精度なし・混在はcompile error |
| 実行環境 | Linux x86_64でテスト。Windows/macOS・他archは未検証 |
| 基準形状 | 非均等ポリライン、折れ点保持、必須sample・名前付き材料位置、剛体配置 |
| 質量・接触 | 線密度から質量を計算。body内で共通の半径・摩擦・collision group |
| 拘束 | pin、MovePin、剛体local anchorへのattach、解放時の速度保持 |
| ハーネス | 接続した木構造、共有junction、span別線密度・spring設定・材料位置、全体の登録/削除 |
| 出力 | 中心線、snapshot、ひずみ・離散曲率・推定曲げ半径・anchor誤差、再生JSONとviewer |
| 未対応 | 閉ループ、junction姿勢・cross-span曲げ、束内滑り、ねじり、断面姿勢clamp、滑るガイド、巻き取り、切断、塑性、solver checkpoint |

## 時間と寿命

アプリケーションがRapier worldとstepを所有します。WorldIdはworldごとに一意な値を割り当て、prepare → world.step() → inspectの順で呼びます。step番号は呼び出し順の検査用で、実際のworld進行回数を証明するものではありません。

IDはsetと世代に結び付きます。別set、削除済みID、native物体の外部削除やトポロジー変更、管理外の拘束変更はエラーになります。全コマンドの検証失敗時は操作を適用しません。inspectの失敗は、既に進んだworldをrollbackしません。Dropはworldから物体を削除しないため、removeを明示してください。

## 数値と物性

周波数・減衰比はnative springの設定であり、実測の弾性率ではありません。samplingを変えたときに同じ物理剛性を保つ校正はしていません。細分化によって伸びが増える場合もあります。分割数、時間刻み、solver反復、重力、物性設定を組にして評価してください。

任意の寸法・速度での非貫通、結び目や絡みの保持、実物との一致、リアルタイム性能は保証しません。自己接触と枝接触はRapierのwire接触・隣接除外規則に従います。ハーネスの接合部専用の接触モデルは持ちません。

## 診断と記録

曲率と曲げ半径は中心線の離散推定です。端点・直線・零長区間では理由付きの未定義値を返します。非有限値、未定義、未対応、未提供は別の状態です。ハーネスの幾何はspan内で計算し、junctionの曲率を定義しません。

native edge/attachment impulseは最後の内部substepの値で、単位はN sです。outer step全体の和や平均張力ではありません。材料応力、EA/EI/GJへの校正、ねじり、solver収束は未提供です。

RopeTrackのschema version 1とYハーネス例のgraph schema version 1は別の実験用再生形式です。永続互換やsolver復元を保証する形式ではありません。時刻と操作イベントは利用側が記録します。

## 依存とライセンス

本crateはMITです。RapierとParryはApache-2.0で、依存のライセンスはそれぞれに従います。[依存一覧](../THIRD_PARTY_NOTICES.md)を同梱しています。serde_jsonのfloat_roundtripを有効にし、f64の記録値を正確に読み戻します。

[ロープ利用ガイド](usage.ja.md) · [ハーネス利用ガイド](harness.ja.md)
