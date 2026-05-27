# ROS2-TMS-for-Construction — Docker 開発環境（X11 版）

`main` を派生元とした Docker 開発環境。`docker/` サブディレクトリ配下だけで完結しており、既存ファイルには手を入れない。

- **ベース image**: `osrf/ros:humble-desktop-full`（Ubuntu 22.04 jammy）
- **GUI**: X11 forward（VNC なし）
- **MongoDB**: 別サービス (`mongo:6.0`) として分離
- **外部依存**: `docker/src.repos` で commit を固定して vcstool 管理

## Quick Start

### 環境構築

```bash
# cd ros2_tms_for_construction/docker
UID=$(id -u) GID=$(id -g) docker compose build      # 初回 20-30 分
xhost +local:                                        # GUI 用 X 許可（セッションごと）
docker compose up -d                                  # 初回 colcon build (10 分)
docker compose exec tms restore-db.sh                 # DB seed 投入（初回のみ）
```

前提・ワークスペース構造・各ステップの解説は [docs/setup.md](docs/setup.md) を参照。

### 動作確認 (task_id=4)

Unity ([pwri-opera/OperaSim-PhysX](https://github.com/pwri-opera/OperaSim-PhysX)) を別途起動した状態で 2 ターミナルで実行:

Terminal 1: Unity ↔ ROS 2 ブリッジ
```bash
./scripts/exec.sh ros2 launch ros_tcp_endpoint endpoint.py
```

(Unity を再生して接続を確認してから Terminal 2 を起動する)

Terminal 2: bringup (zx200 / tms_if / tms_ts を 3 段連鎖起動)
```bash
./scripts/exec.sh ros2 launch /workspace/src/ros2_tms_for_construction/docker/launch/bringup.launch.yaml
```

Unity 設定、RViz の初期姿勢回避、緑ボタン押下までの詳細手順は [docs/usage.md](docs/usage.md) 参照。

### 停止

```bash
docker compose down         # コンテナ停止（named volume は保持）
# docker compose down -v    # named volume ごと削除（DB・build キャッシュも消える）
```

## ドキュメント

- [docs/setup.md](docs/setup.md) — 前提・ワークスペース構造・起動手順（詳細）
- [docs/usage.md](docs/usage.md) — `task_id=4` 完走手順（Terminal 1 / 2、Unity 設定、RViz 操作、緑ボタン）
- [docs/known-issues.md](docs/known-issues.md) — 既知の制約・トラブルシュート

## `docker/` 配下ファイル

| ファイル | 役割 |
|---|---|
| `Dockerfile` | ベース image + ROS 2 依存 + source build (BehaviorTree.CPP / mongocxx / mongo-c-driver) + `vcs import` |
| `Dockerfile.dockerignore` | このビルド専用の ignore ファイル（BuildKit の per-Dockerfile ignore）。allowlist 形式でビルドコンテキストを絞る |
| `compose.yaml` | `mongodb`（`mongo:6.0`）と `tms` の 2 サービス、named volume、X11 forward、`network_mode: host`、`tms` には Fast DDS の SHM lock 用に `shm_size: 1g` を割当 |
| `src.repos` | vcstool 管理。外部 repo を 40 桁 full commit SHA で pin（コメントで元ブランチと日付を保持） |
| `launch/bringup.launch.yaml` | Terminal 2 用。`zx200_bringup` → `tms_if_for_opera` → `tms_ts_construction` の 3 launch を timer 連鎖起動する YAML launch |
| `scripts/entrypoint.sh` | container 起動時に root で named volume 所有権を修正後、`gosu` で `ros` に drop、成功 sentinel で初回 `colcon build` を一度だけ実行 |
| `scripts/restore-db.sh` | `demo/rostmsdb_collections.zip` を展開して `mongorestore`、`parameter` collection から `description` (string) フィールドを除去（subtask が数値型のみ対応のため） |
| `scripts/exec.sh` | host 側。`docker compose exec --user ros tms bash -c 'source ... && exec "$@"'` の wrapper、DISPLAY 自動 pass |
| `scripts/shell.sh` | host 側。`docker compose exec --user ros tms bash` で対話 shell に入る |
