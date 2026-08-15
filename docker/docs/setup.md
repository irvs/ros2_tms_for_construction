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
- **`restore-db.sh`**: `demo/rostmsdb_collections.zip` を展開して `mongorestore`、続いて `parameter` collection から `description` (string) フィールドを除去（subtask 側の型不整合 workaround）。完了すると `rostmsdb` に task 11 件・parameter 40 件ほどが投入される。動作確認に使う `task_id=1`（zx200 掘削積込タスク）はシードに含まれているため別途登録不要。

## 2 回目以降の起動

```bash
# cd ros2_tms_for_construction/docker
xhost +local:           # 新規ログインセッションなら必要
docker compose up -d    # sentinel があるので colcon build は skip、秒で立ち上がる
```

`src.repos` / `Dockerfile` を更新したときは `docker compose down -v` で named volume ごと削除してから build → up する（`down` だけでは volume が残り古い artifact が再利用される）。

## RMW (DDS) の選択と切替

デフォルトは Cyclone DDS (`rmw_cyclonedds_cpp`)。両 RMW 実装は image に同梱済み (`ros-humble-rmw-{fastrtps,cyclonedds}-cpp`) で、ホスト側に DDS をインストールする必要はない（RMW プラグインはコンテナ内 ROS 2 プロセスに linked-in される）。

| RMW | 起動コマンド | 備考 |
|---|---|---|
| Cyclone DDS (default) | `docker compose up -d` | 同梱 `cyclonedds.xml` (loopback-only + `DontRoute`) を既定適用し単一ホスト内に隔離 |
| Fast DDS | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp docker compose up -d` | `shm_size: 1g` で共有メモリ転送を有効化済み |

### discovery のスコープ

既定の `cyclonedds.xml` は loopback (`127.0.0.1`) のみで discovery するため、**同一ホスト上の他 ROS 2 プロセス（`network_mode: host` の別コンテナ含む）とだけ discover し、LAN 上の別マシンとは繋がらない**。

- 複数マシンで discover したい: `CYCLONEDDS_URI=` で空にして built-in default (host NIC の multicast) に戻す。
- 独自設定を使う: `docker/cyclonedds.xml` を直接編集するか、`CYCLONEDDS_URI=file:///path/to/your.xml` で別ファイルを指す（compose.yaml で pass-through 済み）。
- ホスト内でさらに分離したい: `ROS_DOMAIN_ID` を変更する。

### Zenoh で複数現場を繋ぐ場合

WAN / NAT 越し・複数現場を繋ぎたい場合は、`rmw_zenoh` をコンテナに入れるのではなく、ホストで [`zenoh-bridge-ros2dds`](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) を起動して DDS を Zenoh network に bridge する構成を推奨する（コンテナ側は DDS のままで OK）。`rmw_zenoh` 同梱の `rmw_zenohd` は rmw_zenoh ノード専用 router で DDS との bridge を持たないため、この用途には使えない。

## 次のステップ

- 動作確認 (`task_id=1` 完走): [usage.md](usage.md)
- 既知の制約 / トラブルシュート: [known-issues.md](known-issues.md)
