# ROS2-TMS-for-Construction — Docker 開発環境（X11 版）

`main` を派生元とした Docker 開発環境。`docker/` サブディレクトリ配下だけで完結しており、既存ファイルには手を入れない。

- **ベース image**: `osrf/ros:humble-desktop-full`（Ubuntu 22.04 jammy）
- **GUI**: X11 forward（VNC なし）
- **MongoDB**: 別サービス (`mongo:6.0`) として分離
- **外部依存**: `docker/src.repos` で commit を固定して vcstool 管理

## 前提

- Ubuntu ホスト（ROS 2 humble の Tier 1 プラットフォーム要件）
- Docker Engine + docker-compose v2
- X サーバが稼働中（GUI を使う場合）

## ワークスペース構造

```
/workspace/                          # コンテナ内
├── src/
│   ├── ros2_tms_for_construction/   # host repo を bind mount
│   ├── Groot/                       # 以下 src.repos で clone（image 内）
│   ├── tms_if_for_opera/
│   └── opera/{common,simulator,zx200,ic120}/…
├── build/    ← named volume `tms_build`
├── install/  ← named volume `tms_install`
└── log/      ← named volume `tms_log`
```

## 起動手順

### 1. ビルド

```bash
cd docker
UID=$(id -u) GID=$(id -g) docker compose build
```

`UID` / `GID` を build args で渡すのは、コンテナ内 `ros` ユーザーの UID/GID をホストと揃えて bind mount の権限不一致を防ぐため。初回は BehaviorTree.CPP / mongo-c-driver / mongo-cxx-driver などのソースビルドが走るため 20〜30 分かかる想定。

### 2. X サーバーへのアクセス許可（GUIを使用するセッションごとに 1 回）

```bash
xhost +local:
```

### 3. 起動

```bash
docker compose up -d
```

初回起動時にコンテナ内で `colcon build` が自動実行される（37 packages、10 分前後）。

`src.repos` / `Dockerfile` を更新したときは `docker compose down -v` で named volume ごと削除してから build → up する（`down` だけでは volume が残り古い artifact が再利用される）。

### 4. MongoDB シードデータの投入（初回のみ）

`demo/rostmsdb_collections.zip` を展開して `mongorestore` する:

```bash
docker compose exec tms /workspace/src/ros2_tms_for_construction/docker/restore-db.sh
```

完了すると `rostmsdb` に task 11 件・parameter 40 件ほどが投入される。スクリプト末尾でシードの `parameter` collection から `description` (string) フィールドを自動除去している（subtask 側の型不整合 workaround）。

動作確認に使う `task_id=4`（zx200 掘削積込タスク）はシードに含まれているため `task_generator.py` での登録は不要。

## 動作確認

各ターミナル共通の前置き（sourced shell に入る）:

```bash
docker exec -it -u ros -e DISPLAY=$DISPLAY ros2_tms_dev bash
```

`-u ros` は必須（コンテナの `USER` が root のまま、entrypoint 内で `gosu` drop しているため）。

### Step A. 2 ターミナル起動

各ターミナルで前置きの `docker exec ...` を実行してから以下を起動する。**Terminal 1 を起動して Unity を再生したあとに Terminal 2 を起動する** 順序を守ること（理由は後述）。

| Terminal | 役割 | コマンド |
|---|---|---|
| 1 | Unity ↔ ROS 2 ブリッジ | `ros2 launch ros_tcp_endpoint endpoint.py` |
| 2 | zx200 MoveIt2 + RViz / `tms_if_for_opera` / `tms_ts_construction` を順次起動 | `ros2 launch /workspace/src/ros2_tms_for_construction/docker/launch/bringup.launch.yaml` |

Terminal 2 で起動する `bringup.launch.yaml` は内部で 3 つの launch ファイルを timer 付きで連鎖起動する:

1. **t = 0 s** — `zx200_bringup vehicle.launch.py`（`command_interface_name:=velocity`、`use_rviz:=true` がデフォルト）
2. **t = `tms_if_delay` s（既定 5）** — `tms_if_for_opera tms_if_for_opera.launch.py`
3. **t = `tms_ts_delay` s（既定 10）** — `tms_ts_launch tms_ts_construction.launch.py`（`task_id:=4` がデフォルト）

時間差を入れているのは `zx200_bringup` が `robot_description_semantic` (SRDF) を publish する前に `tms_if_for_opera` 側の MoveGroupInterface が subscribe すると 10 秒タイムアウトで FATAL 終了するため。低性能ホストで FATAL が出る場合は後述の `tms_if_delay` / `tms_ts_delay` を増やす。

Terminal 1 を先に起動する理由: Unity 側の `JointStatePublisher` が `/zx200/joint_states` を publish する前に `zx200_bringup` の `ros2_control` を起動すると、初期姿勢を取得できずコントローラ初期化が不安定になる。Terminal 1 の `ros_tcp_endpoint` を立ててから Step B で Unity を再生し、joint_states が流れ始めてから Terminal 2 を起動する。

利用可能な引数（任意）:

| 引数 | デフォルト | 説明 |
|---|---|---|
| `task_id` | `4` | 緑ボタンで実行する BT の task_id |
| `command_interface_name` | `velocity` | ros2_control の command interface (`velocity` / `position` / `effort`) |
| `use_rviz` | `true` | `zx200_bringup` の RViz を起動するか |
| `tms_if_delay` | `5.0` | `tms_if_for_opera` 起動までの待ち時間（秒）。SRDF の subscribe timeout 回避用 |
| `tms_ts_delay` | `10.0` | `tms_ts_construction` 起動までの待ち時間（秒）。`tms_if_delay` より大きく |

例:

```bash
# 別の task を試す
ros2 launch <…>/bringup.launch.yaml task_id:=5

# 低性能ホストで SRDF subscribe timeout が出る場合は遅延を伸ばす
ros2 launch <…>/bringup.launch.yaml tms_if_delay:=8 tms_ts_delay:=14
```

Terminal 2 起動後（`tms_ts_delay` 経過以降）、緑ボタンを押す前に RViz の MotionPlanning パネルから手動 Plan & Execute も可能（BT を使わない動作確認）。

### Step B. Unity 側の設定（初回のみ）と再生

ROS-TCP-Endpoint (Terminal 1) が listen 状態になってから Unity を再生する。Endpoint が立っていない状態で Unity を再生しても接続できない。

1. Unity 起動 → 上部ツールバー **`Robotics` → `ROS Settings`** を開き、`ROS IP Address` に **ホスト (Docker を実行しているマシン) の IP**、`ROS Port` に **`10000`** を設定。`network_mode: host` でコンテナとホストの NW を共有しているため、コンテナ IP ではなくホスト IP を指す（参考: [pwri-opera/OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX)）
2. `Hierarchy` で `zx200` 選択 → `Inspector` の **`JointStatePublisher / Topic Name`** を `zx200/joint_states` に変更
3. `zx200` 配下を展開（`base_link` → `body_link`） → **`boom_link` / `arm_link` / `bucket_link`** の `Inspector` で `Control Type Annotation / Control Type` が `Position` の場合は **`Velocity`** に変更
4. Unity を **再生**（▶︎）→ Terminal 1 に `Connection from <Unity-IP>` のログが出れば接続成功

### Step C. RViz で初期姿勢を回避（衝突回避の前処理）

Terminal 2 で起動した RViz2 の MoveIt パネルを操作する。

起動直後はバックホウのアームが伸び切った姿勢で、バケットが地面に接触している（バケットが**ピンク色**で表示される＝衝突状態）。この状態のまま緑ボタンを押すと BT 実行中に planning が破綻し表示が崩れるため、**先に手動で 1 回だけ姿勢を正しておく**:

1. RViz の MoveIt MotionPlanning パネル → **`Joints` タブ**
2. **`boom_joint` のスライダーを少し下げる**（角度を負方向に動かしてバケットを地面から浮かす）
3. **`Planning` タブ → `Plan and Execute`** で実行
4. バケットがピンク色でなくなったことを確認

Terminal 2 を起動するたびに必要（停止 → 再起動でこの状態に戻る）。

### Step D. `tms_ur_button` GUI の緑ボタンで BT 実行

Terminal 2 起動から `tms_ts_delay` 秒（既定 10）経過後、`tms_ts_construction` 側の `tms_ur_button` が Tkinter のウィンドウを表示する。緑ボタンを押すと `task_id`（デフォルト `4`）の Behavior Tree が `task_schedular_manager` に送られ、Unity 上のバックホウが掘削動作する。

1 回の実行で BT 全体が SUCCESS して終端する設計のため、**もう一度動かしたい場合は Terminal 2 を Ctrl-C → 再起動** してから Step C → 緑ボタンを押す（または BT 自体を `Repeat num_cycles=N` 構造で登録し直す。シードの `task_id=5` は task_id=4 を 4 回反復する版）。

## 停止

```bash
docker compose down         # コンテナ停止（named volume は保持）
docker compose down -v      # named volume ごと削除（DB・build キャッシュも消える）
```

## 既知の制約

- ROS 2 humble の Tier 1 サポートは Ubuntu 22.04 のみ。他 OS は未サポート。
- GPU アクセラレーションは未設定（必要なら `compose.yaml` に `deploy.resources.reservations.devices` を追加）。
- `*moveit*` / `*nav2*` / `*tf*` をワイルドカードで apt install しているため image サイズが大きい（約 12 GB）。必要パッケージに絞るには Dockerfile を調整する。
- **ポート衝突**: `network_mode: host` のため `27017`（MongoDB）および Terminal 1 で使う `10000`（ros_tcp_endpoint）がホストで使用中だと起動しない。`ss -ltnp` 等で事前確認する。
- **`xhost +local:` は開きっぱなしにしない**: 外部ユーザーもローカル X サーバに接続できる状態になる。作業終了後は `xhost -local:` で閉じる。
- **xrdp 等のリモートデスクトップ経由で `DISPLAY` 値がセッションごとに変わる環境**: `docker compose up` 時の `$DISPLAY` がコンテナ内に固定化される。再接続後は `docker compose down && up` するか `docker exec -u ros -e DISPLAY=$DISPLAY …` で上書きする。
- **MongoDB へのアクセス**: `27017:27017` をホストに publish しているため、Compass 等の MongoDB クライアントから `mongodb://localhost:27017` で接続可（コンテナ内には Compass は入っていない）。

## `docker/` 配下ファイル

| ファイル | 役割 |
|---|---|
| `Dockerfile` | ベース image + ROS 2 依存 + source build で BehaviorTree.CPP / mongocxx / mongo-c-driver + `vcs import` |
| `Dockerfile.dockerignore` | このビルド専用の ignore ファイル（BuildKit の per-Dockerfile ignore）。allowlist 形式でビルドコンテキストを絞る |
| `compose.yaml` | `mongodb`（`mongo:6.0`）と `tms` の 2 サービス、named volume、X11 forward、`network_mode: host`、`tms` には Fast DDS の SHM lock 用に `shm_size: 1g` を割当 |
| `entrypoint.sh` | root で named volume 所有権を修正後、`gosu` で `ros` に drop。成功 sentinel で初回 `colcon build` を一度だけ実行 |
| `restore-db.sh` | `demo/rostmsdb_collections.zip` を展開して `mongorestore`、その後 `parameter` collection から `description` (string) フィールドを除去（subtask が数値型のみ対応のため） |
| `src.repos` | vcstool 管理。外部 repo を 40 桁 full commit SHA で pin（コメントで元ブランチと日付を保持） |
| `launch/bringup.launch.yaml` | Terminal 2 用。`zx200_bringup` → 8 s → `tms_if_for_opera` → 14 s → `tms_ts_construction` の 3 launch を timer 連鎖起動する YAML launch |
