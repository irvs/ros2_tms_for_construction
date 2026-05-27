# Known Issues

`docker/` 開発環境の既知の制約とトラブルシュート。

## プラットフォーム / image

- ROS 2 humble の Tier 1 サポートは Ubuntu 22.04 のみ。他 OS は未サポート。
- GPU アクセラレーションは未設定（必要なら `compose.yaml` に `deploy.resources.reservations.devices` を追加）。
- `*moveit*` / `*nav2*` / `*tf*` をワイルドカードで apt install しているため image サイズが大きい（約 12 GB）。必要パッケージに絞るには Dockerfile を調整する。

## ポート衝突 (`network_mode: host`)

`27017`（MongoDB）および Terminal 1 で使う `10000`（ros_tcp_endpoint）がホストで使用中だと起動しない。事前確認:

```bash
ss -ltnp | grep -E ':27017|:10000'
```

特にホストに ROS2 TMS for Construction をネイティブで構築済みの環境では、**`mongod` が `systemd` で常駐していて気づきにくい** ことがある:

```bash
sudo systemctl stop mongod
sudo systemctl disable mongod   # 自動起動も止める
```

停止せずに `restore-db.sh` を実行すると、ネイティブの MongoDB にシードを誤投入してしまう。

## X11 forward

`xhost +local:` は開きっぱなしにしない（外部ユーザーもローカル X サーバに接続できる状態になる）。作業終了後:

```bash
xhost -local:
```

### xrdp 等のリモートデスクトップで `$DISPLAY` がセッションごとに変わる環境

`docker compose up` 時点の `$DISPLAY` がコンテナ内に固定化される。再接続後は `./scripts/exec.sh` / `./scripts/shell.sh` を使えば呼び出し時の `$DISPLAY` が毎回 container に渡されるため OK。強制更新したい場合:

```bash
# cd ros2_tms_for_construction/docker
docker compose down && docker compose up -d
```

## MongoDB へのアクセス

`27017:27017` をホストに publish しているため、Compass 等の MongoDB クライアントから `mongodb://localhost:27017` で接続可（コンテナ内には Compass は入っていない）。
