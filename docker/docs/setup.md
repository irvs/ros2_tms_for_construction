# Setup

`docker compose` で本リポジトリの開発環境を立ち上げるまでの手順。

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

`src/ros2_tms_for_construction/` のみホストと bind mount で双方向に編集可能。他の clone は image 内で固定 (編集には `docker/src.repos` の pin 書き換え → `docker compose down -v` → rebuild が必要)。複数 repo を並行編集したい場合は、外部依存も host 側に展開する meta-workspace 形式の方が向く。

## 起動手順 (初回環境構築)

ホスト側で `docker/` ディレクトリに移動してから 1 ターミナルで連続実行する:

```bash
# cd ros2_tms_for_construction/docker
UID=$(id -u) GID=$(id -g) docker compose build      # 初回 20-30 分
xhost +local:                                        # GUI 用 X 許可（セッションごと）
docker compose up -d                                  # 初回 colcon build (10 分)
docker compose exec tms restore-db.sh                 # DB seed 投入（初回のみ）
```

### 各 step の補足

- **`UID` / `GID` を build args で渡す**: コンテナ内 `ros` ユーザーの UID/GID をホストと揃え、bind mount の権限不一致を防ぐ。初回は BehaviorTree.CPP / mongo-c-driver / mongo-cxx-driver などの source build が走るため 20〜30 分かかる想定。
- **`xhost +local:`**: GUI（RViz など）を表示するための X 許可。セッションごと 1 回。作業終了後は `xhost -local:` で閉じる（[known-issues.md](known-issues.md) 参照）。
- **`docker compose up -d`**: 初回起動時にコンテナ内で `colcon build --symlink-install` が自動実行される（37 packages、10 分前後）。`/workspace/install/.colcon_build_succeeded` sentinel でガードしており、2 回目以降は skip。
- **`restore-db.sh`**: `demo/rostmsdb_collections.zip` を展開して `mongorestore`、続いて `parameter` collection から `description` (string) フィールドを除去（subtask 側の型不整合 workaround）。完了すると `rostmsdb` に task 11 件・parameter 40 件ほどが投入される。動作確認に使う `task_id=4`（zx200 掘削積込タスク）はシードに含まれているため別途登録不要。

## 2 回目以降の起動

```bash
# cd ros2_tms_for_construction/docker
xhost +local:           # 新規ログインセッションなら必要
docker compose up -d    # sentinel があるので colcon build は skip、秒で立ち上がる
```

`src.repos` / `Dockerfile` を更新したときは `docker compose down -v` で named volume ごと削除してから build → up する（`down` だけでは volume が残り古い artifact が再利用される）。

## 次のステップ

- 動作確認 (`task_id=4` 完走): [usage.md](usage.md)
- 既知の制約 / トラブルシュート: [known-issues.md](known-issues.md)
