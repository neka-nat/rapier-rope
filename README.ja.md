# rapier-rope

Rapier 0.36のsoft bodyを使い、ロープ・ケーブル・木構造ハーネスの生成、固定・把持・解放、診断を扱うRustパッケージです。worldと時間進行はアプリケーションが所有します。

[English](README.md) · [ロープ利用ガイド](docs/usage.ja.md) · [ハーネス利用ガイド](docs/harness.ja.md) · [対応と制限](docs/compatibility.ja.md)

## 導入

Rust 1.90以上、3D、CPUに対応します。既定の精度はf32です。

```toml
[dependencies]
rapier-rope = "0.1.0"
```

f64を使う場合:

```toml
rapier-rope = { version = "0.1.0", default-features = false, features = ["f64"] }
```

両精度の同時指定と、精度なしの指定はできません。native型には`rapier_rope::rapier`のre-exportを使うと、Rapierのversionと精度を揃えられます。

## 主な機能

- 折れ点と名前付き材料位置を保持するsampling、線密度からの質量計算
- world点への固定と移動、剛体への点接続、把持・解放
- 世代付きID、削除・外部編集の検査、prepare → step → inspectの手順
- 共有junctionとspan別の材料設定を持つ木構造ハーネス
- 中心線snapshot、ひずみ・曲率・anchor誤差、JSON再生とviewer

生成例は[Quick start](README.md#quick-start)、生成から解放・削除までのコードは[利用ガイド](docs/usage.ja.md)を参照してください。

## 再生例

ソースを取得したディレクトリで実行します。

```bash
cargo run --release --example record_tracks -- target/tracks/f32
cargo run --release --example y_harness -- target/tracks/f32/y-harness.json
```

[viewer](tools/view_track.html)をブラウザで開き、「JSONを開く」から生成したファイルを選択します。中心線、接触半径、固定点、取り付けanchor、コネクタを表示します。保存データは再生用で、solverの完全再開には使えません。

## モデルの制限

周波数・減衰比は実測のEA/EI/GJと同一ではありません。ねじり、断面姿勢固定、滑るガイド、巻き取り、切断、塑性は未対応です。ハーネスは接続した木構造と固定トポロジーに限定し、接触設定をbody内で共通にします。junctionの姿勢拘束やspanを跨ぐ曲げ抵抗はありません。

分割数、刻み、solver反復によって伸びや接触の結果が変わります。平均張力・材料応力は未提供です。[対応と制限](docs/compatibility.ja.md)を確認してください。

MITライセンスです。[LICENSE](LICENSE)と[依存ライセンス一覧](THIRD_PARTY_NOTICES.md)を参照してください。
