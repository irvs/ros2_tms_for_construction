#!/usr/bin/env bash
#
# Host から container に対話 bash で入る。
# .bashrc で /opt/ros/humble/setup.bash + /workspace/install/setup.bash が
# source されるので、入った直後から ros2 / colcon が使える。
#
# 使い方:
#   ./scripts/shell.sh
#
# 一発コマンドだけ叩きたいときは scripts/exec.sh を使う。

set -e
cd "$(dirname "$0")/.."

exec docker compose exec \
  ${DISPLAY:+-e DISPLAY=$DISPLAY} \
  --user ros tms bash
