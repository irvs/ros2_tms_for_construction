# Known Issues

`docker/` 開発環境の既知の制約とトラブルシュート。

- ROS 2 humble の Tier 1 サポートは Ubuntu 22.04 のみ。他 OS は未サポート。
- GPU アクセラレーションは未設定（必要なら `compose.yaml` に `deploy.resources.reservations.devices` を追加）。
- `*moveit*` / `*nav2*` / `*tf*` をワイルドカードで apt install しているため image サイズが大きい（約 12 GB）。必要パッケージに絞るには Dockerfile を調整する。
- **ポート衝突**: `network_mode: host` のため `27017`（MongoDB）および Terminal 1 で使う `10000`（ros_tcp_endpoint）がホストで使用中だと起動しない。`ss -ltnp` 等で事前確認する。
  - 特にホストに ROS2 TMS for Construction をネイティブで構築済み環境では、**`mongod` が `systemd` で常駐していて気づきにくい** ことがある。`sudo systemctl stop mongod`（必要なら `sudo systemctl disable mongod`）で停止する。停止せずに `restore-db.sh` を実行すると、ネイティブの MongoDB にシードを誤投入してしまう。
- **`xhost +local:` は開きっぱなしにしない**: 外部ユーザーもローカル X サーバに接続できる状態になる。作業終了後は `xhost -local:` で閉じる。
- **xrdp 等のリモートデスクトップ経由で `DISPLAY` 値がセッションごとに変わる環境**: `docker compose up` 時の `$DISPLAY` がコンテナ内に固定化される。再接続後は `docker compose down && up` するか `./scripts/exec.sh` / `./scripts/shell.sh` を使う（呼び出し時の `$DISPLAY` を毎回 container に渡すので、ホスト側 shell の DISPLAY が正しければ OK）。
- **MongoDB へのアクセス**: `27017:27017` をホストに publish しているため、Compass 等の MongoDB クライアントから `mongodb://localhost:27017` で接続可（コンテナ内には Compass は入っていない）。
