# Usage

Unity ([pwri-opera/OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX)) を別途起動した状態で、Docker コンテナ内の ROS 2 ノードと連動させて `task_id=4`（zx200 掘削積込タスク）を完走させる手順。

ホストから container を叩く方法は 2 通り:

```bash
./scripts/exec.sh <cmd …>   # 1 コマンド実行 (ROS env source 済、DISPLAY 自動 pass)
./scripts/shell.sh           # 対話 bash で入る (.bashrc で ROS env source)
```

これまで使っていた `docker exec -it -u ros -e DISPLAY=$DISPLAY ros2_tms_dev bash` の長文を 1 行に短縮したもの。`docker exec` を直接叩くと entrypoint をバイパスして `/opt/ros/humble/setup.bash` と `/workspace/install/setup.bash` が source されず、自前の msg / pkg が見えなくなる落とし穴がある。

## Step A. 2 ターミナル起動

**Terminal 1 を起動して Unity を再生したあとに Terminal 2 を起動する** 順序を守ること（理由は後述）。

Terminal 1: Unity ↔ ROS 2 ブリッジ

```bash
# cd ros2_tms_for_construction/docker
./scripts/exec.sh ros2 launch ros_tcp_endpoint endpoint.py
```

Terminal 2: zx200 MoveIt2 + RViz / `tms_if_for_opera` / `tms_ts_construction` を順次起動

```bash
# cd ros2_tms_for_construction/docker
./scripts/exec.sh ros2 launch /workspace/src/ros2_tms_for_construction/docker/launch/bringup.launch.yaml
```

Terminal 2 で起動する `bringup.launch.yaml` は内部で 3 つの launch ファイルを timer 付きで連鎖起動する:

1. **t = 0 s** — `zx200_bringup vehicle.launch.py`（`command_interface_name:=velocity`、`use_rviz:=true` がデフォルト）
2. **t = `tms_if_delay` s（既定 5）** — `tms_if_for_opera tms_if_for_opera.launch.py`
3. **t = `tms_ts_delay` s（既定 10）** — `tms_ts_launch tms_ts_construction.launch.py`（`task_id:=4` がデフォルト）

時間差を入れているのは `zx200_bringup` が `robot_description_semantic` (SRDF) を publish する前に `tms_if_for_opera` 側の MoveGroupInterface が subscribe すると 10 秒タイムアウトで FATAL 終了するため。低性能ホストで FATAL が出る場合は後述の `tms_if_delay` / `tms_ts_delay` を増やす。

Terminal 1 を先に起動する理由: Unity 側の `JointStatePublisher` が `/zx200/joint_states` を publish する前に `zx200_bringup` の `ros2_control` を起動すると、初期姿勢を取得できずコントローラ初期化が不安定になる。Terminal 1 の `ros_tcp_endpoint` を立ててから Step B で Unity を再生し、joint_states が流れ始めてから Terminal 2 を起動する。

### bringup の引数（任意）

| 引数 | デフォルト | 説明 |
|---|---|---|
| `task_id` | `4` | 緑ボタンで実行する BT の task_id |
| `command_interface_name` | `velocity` | ros2_control の command interface (`velocity` / `position` / `effort`) |
| `use_rviz` | `true` | `zx200_bringup` の RViz を起動するか |
| `tms_if_delay` | `5.0` | `tms_if_for_opera` 起動までの待ち時間（秒）。SRDF の subscribe timeout 回避用 |
| `tms_ts_delay` | `10.0` | `tms_ts_construction` 起動までの待ち時間（秒）。`tms_if_delay` より大きく |

別の task を試す場合:

```bash
./scripts/exec.sh ros2 launch /workspace/src/ros2_tms_for_construction/docker/launch/bringup.launch.yaml task_id:=5
```

低性能ホストで SRDF subscribe timeout が出る場合:

```bash
./scripts/exec.sh ros2 launch /workspace/src/ros2_tms_for_construction/docker/launch/bringup.launch.yaml tms_if_delay:=8 tms_ts_delay:=14
```

Terminal 2 起動後（`tms_ts_delay` 経過以降）、緑ボタンを押す前に RViz の MotionPlanning パネルから手動 Plan & Execute も可能（BT を使わない動作確認）。

## Step B. Unity 側の設定（初回のみ）と再生

ROS-TCP-Endpoint (Terminal 1) が listen 状態になってから Unity を再生する。Endpoint が立っていない状態で Unity を再生しても接続できない。

1. Unity 起動 → 上部ツールバー **`Robotics` → `ROS Settings`** を開き、`ROS IP Address` に **ホスト (Docker を実行しているマシン) の IP**、`ROS Port` に **`10000`** を設定。`network_mode: host` でコンテナとホストの NW を共有しているため、コンテナ IP ではなくホスト IP を指す（参考: [pwri-opera/OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX)）
2. `Hierarchy` で `zx200` 選択 → `Inspector` の **`JointStatePublisher / Topic Name`** を `zx200/joint_states` に変更
3. `zx200` 配下を展開（`base_link` → `body_link`） → **`boom_link` / `arm_link` / `bucket_link`** の `Inspector` で `Control Type Annotation / Control Type` が `Position` の場合は **`Velocity`** に変更
4. Unity を **再生**（▶︎）→ Terminal 1 に `Connection from <Unity-IP>` のログが出れば接続成功

## Step C. RViz で初期姿勢を回避（衝突回避の前処理）

Terminal 2 で起動した RViz2 の MoveIt パネルを操作する。

起動直後はバックホウのアームが伸び切った姿勢で、バケットが地面に接触している（バケットが**ピンク色**で表示される＝衝突状態）。この状態のまま緑ボタンを押すと BT 実行中に planning が破綻し表示が崩れるため、**先に手動で 1 回だけ姿勢を正しておく**:

1. RViz の MoveIt MotionPlanning パネル → **`Joints` タブ**
2. **`boom_joint` のスライダーを少し下げる**（角度を負方向に動かしてバケットを地面から浮かす）
3. **`Planning` タブ → `Plan and Execute`** で実行
4. バケットがピンク色でなくなったことを確認

Terminal 2 を起動するたびに必要（停止 → 再起動でこの状態に戻る）。

## Step D. `tms_ur_button` GUI の緑ボタンで BT 実行

Terminal 2 起動から `tms_ts_delay` 秒（既定 10）経過後、`tms_ts_construction` 側の `tms_ur_button` が Tkinter のウィンドウを表示する。緑ボタンを押すと `task_id`（デフォルト `4`）の Behavior Tree が `task_schedular_manager` に送られ、Unity 上のバックホウが掘削動作する。

1 回の実行で BT 全体が SUCCESS して終端する設計のため、**もう一度動かしたい場合は Terminal 2 を Ctrl-C → 再起動** してから Step C → 緑ボタンを押す（または BT 自体を `Repeat num_cycles=N` 構造で登録し直す。シードの `task_id=5` は task_id=4 を 4 回反復する版）。

## 停止

```bash
# cd ros2_tms_for_construction/docker
docker compose down         # コンテナ停止（named volume は保持）
# docker compose down -v    # named volume ごと削除（DB・build キャッシュも消える）
```
