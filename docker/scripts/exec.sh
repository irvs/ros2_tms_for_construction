#!/usr/bin/env bash
#
# Host から container 内で ROS 環境付きコマンドを実行するラッパー。
# `docker compose exec --user ros tms bash -c '...'` を 1 行で叩く。
#
# 通常の `docker exec` は entrypoint をバイパスするので /opt/ros/humble/
# setup.bash と /workspace/install/setup.bash が source されず、メッセージ型や
# 自前パッケージが見えない。bash -c の中で source してから "$@" を exec する。
#
# 使い方:
#   ./scripts/exec.sh ros2 node list
#   ./scripts/exec.sh ros2 topic hz /zx200/joint_states
#   ./scripts/exec.sh colcon build --symlink-install --packages-up-to my_pkg
#   ./scripts/exec.sh ros2 launch ros_tcp_endpoint endpoint.py   # GUI 系も
#
# DISPLAY は環境変数がセットされていれば自動で container に渡される (xrdp 等で
# セッションごとに値が変わるケースをカバー)。
#
# 対話 shell に入りたいときは scripts/shell.sh を使う。

set -e
cd "$(dirname "$0")/.."

exec docker compose exec \
  ${DISPLAY:+-e DISPLAY=$DISPLAY} \
  --user ros tms \
  bash -c '
    source /opt/ros/humble/setup.bash
    [ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash
    cd /workspace
    exec "$@"
  ' bash "$@"
