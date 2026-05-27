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

## 起動手順

### 1. ビルド

```bash
cd docker
UID=$(id -u) GID=$(id -g) docker compose build
```

`UID` / `GID` を build args で渡すのは、コンテナ内 `ros` ユーザーの UID/GID をホストと揃えて bind mount の権限不一致を防ぐため。初回は BehaviorTree.CPP / mongo-c-driver / mongo-cxx-driver などのソースビルドが走るため 20〜30 分かかる想定。

### 2. X サーバーへのアクセス許可（GUI を使うセッションごとに 1 回）

```bash
xhost +local:
```

### 3. 起動

```bash
docker compose up -d
```

初回起動時にコンテナ内で `colcon build --symlink-install` が自動実行される（37 packages、10 分前後）。`/workspace/install/.colcon_build_succeeded` sentinel でガードしており、2 回目以降は skip。

`src.repos` / `Dockerfile` を更新したときは `docker compose down -v` で named volume ごと削除してから build → up する（`down` だけでは volume が残り古い artifact が再利用される）。

### 4. MongoDB シードデータの投入（初回のみ）

```bash
docker compose exec tms restore-db.sh
```

`demo/rostmsdb_collections.zip` を展開して `mongorestore`、続いて `parameter` collection から `description` (string) フィールドを除去（subtask 側の型不整合 workaround）。

完了すると `rostmsdb` に task 11 件・parameter 40 件ほどが投入される。動作確認に使う `task_id=4`（zx200 掘削積込タスク）はシードに含まれているため別途登録不要。

## 次のステップ

- 動作確認 (`task_id=4` 完走): [usage.md](usage.md)
- 既知の制約 / トラブルシュート: [known-issues.md](known-issues.md)
