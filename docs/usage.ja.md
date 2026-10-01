# ロープ利用ガイド

Rust 1.90以上、3D、CPU、固定トポロジーのロープを対象とします。worldと時間進行はアプリケーションが所有します。

## 依存と精度

```toml
[dependencies]
rapier-rope = "0.1.0"
```

f64では`rapier-rope = { version = "0.1.0", default-features = false, features = ["f64"] }`を指定します。既定はf32です。両方有効・両方無効はcompile errorとなります。native型には同じversionと精度の`rapier_rope::rapier`を使ってください。

## 生成から解放・削除まで

以下は単独の`src/main.rs`として両精度でビルド・実行するコード。96step中に剛体との接続と固定点を順に解放する。

```rust
use rapier_rope::{rapier::prelude::*, *};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let id = WorldId(1); // アプリ内のworldごとに異なるIDを割り当てる
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 240.0;
    world.integration_parameters.num_solver_iterations = 8;
    let dt = world.integration_parameters.dt as f64;
    let mut ropes = RopeSet::new(id)?;
    let spec = RopeSpec::new(
        "cable", vec![[0.0, 1.5, 0.0], [1.0, 1.5, 0.0]],
        NativeRopeMaterial::new(0.1, SpringSettings::new(500.0, 1.0),
            SpringSettings::new(20.0, 0.8)),
        SamplingSettings::new(1.0 / 32.0), CollisionSettings::new(0.005),
    );
    let rope = ropes.insert(id, &mut world, &spec)?;
    let (body, _) = world.insert(
        RigidBodyBuilder::dynamic().translation(Vector::new(1.0, 1.44, 0.0))
            .can_sleep(false),
        ColliderBuilder::ball(0.05).mass(0.05),
    );
    let mut attachment = None;
    for step in 0..96 {
        let commands = match step {
            0 => vec![
                AttachmentCommand::Pin { rope, location: RopeLocation::Start,
                    position_m: [0.0, 1.5, 0.0] },
                AttachmentCommand::Attach { rope, location: RopeLocation::End, body },
            ],
            48 => vec![AttachmentCommand::Detach { attachment: attachment.unwrap() }],
            64 => vec![AttachmentCommand::Unpin { rope, location: RopeLocation::Start }],
            _ => vec![],
        };
        let prepared = ropes.prepare_attachments(id, &mut world, step, dt, &commands)?;
        if step == 0 {
            attachment = Some(prepared.prepared.created_attachments[0].handle);
        }
        world.step();
        ropes.inspect(id, &world, step)?;
        let stamp = CaptureStamp::new(CapturePhase::AfterStep, Some(step),
            (step + 1) as f64 * dt, dt)?;
        let snapshot = ropes.centerline(id, &world, rope)?.snapshot(stamp)?;
        assert!(snapshot.diagnostics.geometry.current_length_m.value().unwrap() > 0.0);
    }
    ropes.remove(id, &mut world, rope)?;
    assert!(ropes.get(id, &world, rope).is_err()); // 削除後のID
    assert_eq!(world.bodies.len(), 1); // 剛体の寿命はアプリ側で管理
    println!("PASS: guide lifecycle");
    Ok(())
}
```

基準形状・材料定義・材料位置はf64のSI値。生成時にnative精度へ変換し、線密度×区間長から質量を各端へ半分ずつ配る。接触半径は分割数に依存しない。非均等ポリラインの折れ点は保持し、`sampling.required_samples_m`または`named_locations`で必須の材料位置を追加する。

`RopeLocation::Named(name)`または`ArcLength { arc_length_m, tolerance_m }`も使える。変形後のworld位置ではなく基準中心線上の材料位置を指定する。`prepared.locations`が実際の粒子・基準弧長・誤差を返す。許容差外の操作、重複取得、外部編集による不整合は操作前にエラーとなる。

## stepとエラーの扱い

各stepで`prepare_attachments`または`prepare` → アプリの`world.step()` → `inspect`の順に呼ぶ。step番号は0から単調に増やし、dtはworldの実効値と一致させる。`WorldId`はアプリが一意に割り当てる値であり、任意のworldを自動識別する機構ではない。

`Pin`は明示したworld点へ位置を設定して速度をゼロにする。`MovePin`は次stepの目標を指定する。`Attach`は現在の粒子位置から剛体local anchorを作り、点を拘束する。`Detach`と`Unpin`は解放時点の速度を保つ。断面姿勢を固定する把持ではない。

`inspect`がエラーを返す時にはworldは既にstepを終えている。単に同じstepを再試行して状態を戻したことにしない。未確認の状態でロープを使い続けず、アプリが原因の外部削除・編集を扱う。検査は実際のworld.step()の呼び出し回数を証明するものではありません。Dropでworld内の物体は自動削除されないため、removeを明示的に呼びます。

## 保存・表示

`RopeTrack::new` → `register_rope` → `push_frame` → `write_json` / `read_json`で所有snapshotを保存する。記録schemaは実験用version 1。時刻・step番号はアプリが渡し、固定・接続・解放イベントもアプリが実行した操作に合わせて記録する。[外部consumerの完全な例](../tests/consumer-common/shared.rs)が、12frameの実際のJSON読み戻しを確認する。

```bash
cargo run --locked --release --example record_tracks -- target/tracks/f32
python3 -m http.server 8000 --bind 127.0.0.1
```

`http://127.0.0.1:8000/tools/view_track.html`で基準trackを読む。f64は`?base=../target/tracks/f64/`を付ける。viewerを直接開き、JSONファイルを選択する方法でも使える。

中心線・接触半径・固定・anchor・剛体poseを描く。保存データは再生専用でsolverのcheckpointではない。曲率はsample間隔に依存する幾何推定。native impulseは最後の内部substepのN sで、平均張力・材料応力・ねじり等の未提供を数値0として解釈しない。[対応と制限](compatibility.ja.md)も参照してください。


分岐の定義・操作・再生例は[ハーネス利用ガイド](harness.ja.md)を参照してください。
