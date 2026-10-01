# 木構造ハーネスの利用

3D/CPU、f32またはf64、固定トポロジーの接続した木構造を扱います。`HarnessSpec`を一つのsoft-bodyへ生成し、接続するspanの端点は同じ粒子を共有します。`RopeSet`と同様、worldとstepは利用側が所有します。

```rust
use rapier_rope::{rapier::prelude::*, *};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let material = NativeRopeMaterial::new(0.1,
        SpringSettings::new(500.0, 1.0), SpringSettings::new(20.0, 0.8));
    let mut spec = HarnessSpec::new("Y", vec![
        JunctionSpec::new("J", [0.0, 1.0, 0.0]),
        JunctionSpec::new("mount", [0.0, 1.5, 0.0]),
        JunctionSpec::new("a", [-0.5, 0.8, 0.0]),
        JunctionSpec::new("b", [0.5, 0.8, 0.0]),
    ], ["mount", "a", "b"].into_iter().map(|end|
        SpanSpec::new(end, "J", end, material.clone(), SamplingSettings::new(0.05))
    ).collect(), CollisionSettings::new(0.005));
    spec.collision.self_contacts = true;
    spec.spans[1].material.linear_density_kg_m = 0.2;
    spec.spans[1].named_locations.push(NamedLocation::new("clip", 0.2));

    let id = WorldId(1);
    let mut world = PhysicsWorld::new();
    world.integration_parameters.dt = 1.0 / 240.0;
    let mut harnesses = HarnessSet::new(id)?;
    let harness = harnesses.insert(id, &mut world, &spec)?;
    harnesses.prepare(id, &mut world, 0, 1.0 / 240.0, &[HarnessCommand::Pin {
        harness, location: HarnessLocation::Junction("mount".into()),
        position_m: [0.0, 1.5, 0.0],
    }])?;
    world.step();
    harnesses.inspect(id, &world, 0)?;
    let view = harnesses.get(id, &world, harness)?;
    let clip = view.samples.resolve_location(&HarnessLocation::Span {
        span: "a".into(), location: RopeLocation::Named("clip".into()),
    })?;
    println!("clip native particle: {}", clip.particle_index);
    let snapshot = view.snapshot(CaptureStamp::new(
        CapturePhase::AfterStep, Some(0), 1.0 / 240.0, 1.0 / 240.0)?)?;
    assert_eq!(snapshot.span_geometry.len(), 3);
    harnesses.remove(id, &mut world, harness)?;
    Ok(())
}
```

定義の位置はm、質量はkg、時間はsです。`JunctionSpec`はdegree 1の端点も含む名前付き頂点です。`SpanSpec.start/end`が頂点名を参照し、`interior_points_m`には端点を含めず途中の折れ点を与えます。座標が一致する別名頂点は自動で結合しません。閉ループ、自己loop、並列span、非接続graph、孤立頂点、零長区間は拒否します。

各spanは既存のsampling規則を使い、折れ点、必須sample、名前付き材料位置を保持します。`SampledSpan.reference()`のparticle indexはspan内の番号です。`particle_indices()`でnative番号へ変換できます。`SampledSpan.resolve_location()`と`SampledHarness.resolve_location()`はglobal/native番号を返します。弧長と許容誤差はそのspanの基準形状に対する値で、ハーネス全体の弧長は定義しません。

各区間の名目質量`線密度 × 基準長`を両端へ半分ずつ配分し、junction粒子へ全接続spanの寄与を一度ずつ加算します。共有粒子を結ぶ零長weld edgeは生成しません。structural/bendの周波数・減衰比と軸応答はspan別に設定できます。bend edgeはspan内の二つ離れたsampleだけです。異なるspan間の曲げ抵抗やjunction姿勢clampは追加しません。

接触半径・摩擦・group・自己接触、nativeの追加反復・sleep、linear dampingはbody単位です。`SpanSpec.collision`に違う値を要求した場合と、spanごとのlinear dampingが異なる場合は拒否します。粒子数はspanごとのsampling budgetと、共有後の`HarnessSpec.max_particles`の両方で検査します。f32への変換でedgeが潰れる配置等も既存のprecision検査で拒否します。

コネクタは同じworldに利用側が作る通常のdynamic剛体です。`HarnessCommand::Attach`へbodyと材料位置を渡すと、現在の粒子位置からlocal anchorを捕捉します。粒子をteleportせず、コネクタ質量をロープへ加算しません。pin/MovePin/unpin/attach/detachの意味と速度の扱いはロープと同じです。二つのspanから共有endpointを同時に取得する指令は同じ粒子への重複として拒否します。

`HarnessSet`は世代付きの独立した`HarnessHandle`を返し、既存のnative所有・constraint・prepare/inspect検査を内部で共有します。`HarnessError`はharnessのIDと`RopeSetErrorKind`の共通検査kindを返します。全コマンドの検証失敗時は無変更です。inspect失敗はworldのrollbackではありません。全ハーネスのremoveだけを提供し、spanだけの削除や切断・tearingは未対応です。Dropもworldから自動削除しません。

`HarnessSnapshot`はgraph中心線、junction/spanからglobal粒子へのmap、pin、native impulseとanchorを保存します。幾何診断のparticle indexと弧長はspan内の番号です。junctionの曲率・姿勢・ねじりは理由付きの未対応で、平均張力・材料応力・solver収束は未提供です。frequency設定を実物のEA/EI/GJと扱わず、記録をsolver checkpointとも扱いません。

再生例は動的plugを付け、左枝を移動把持して1秒で解放します。

```bash
cargo run --locked --release --example y_harness -- target/tracks/f32/y-harness.json
cargo run --locked --release --no-default-features --features f64 --example y_harness -- target/tracks/f64/y-harness.json
cargo run --locked --release --example record_tracks -- target/tracks/f32
```

`tools/view_track.html`でJSONを開けます。例のgraph schema version 1は再生用の実験形式で、`RopeTrack`のschemaとは別です。チューブは接触半径を表し、分岐を描けることは束内滑り・断面ねじりの証拠になりません。junctionの隣接接触除外は上流のwire規則に従います。接触品質は用途の寸法・速度・分割数で確認してください。

