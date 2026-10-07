#!/usr/bin/env bash
#
# Restore the seed MongoDB collections shipped under demo/rostmsdb_collections.zip
# into the running `mongodb` service. Run this once after `docker compose up -d`.
#
# Usage (from host):
#   docker compose exec tms restore-db.sh
#
# Writes to the bind-mounted demo/ directory (extracts rostmsdb_collections.zip
# into demo/dump/ on every run, overwriting a dump left by an older commit), so
# we drop to the `ros` user if invoked as root. Otherwise demo/dump would end
# up root-owned on the host.

set -euo pipefail

if [ "$(id -u)" = "0" ]; then
    exec gosu ros "$0" "$@"
fi

DEMO_DIR="/workspace/src/ros2_tms_for_construction/demo"
ZIP_PATH="${DEMO_DIR}/rostmsdb_collections.zip"
DUMP_DIR="${DEMO_DIR}/dump"

MONGO_HOST="${MONGO_HOST:-localhost}"
MONGO_PORT="${MONGO_PORT:-27017}"

if [ ! -f "${ZIP_PATH}" ]; then
    echo "[restore-db] ERROR: ${ZIP_PATH} not found" >&2
    exit 1
fi
# Always re-extract: every version of the zip unpacks to the same file names
# under dump/rostmsdb/, so skipping when dump/ exists would silently restore
# an older seed after the zip is updated.
echo "[restore-db] Extracting ${ZIP_PATH}"
(cd "${DEMO_DIR}" && unzip -o -q "$(basename "${ZIP_PATH}")")

echo "[restore-db] mongorestore --drop --host ${MONGO_HOST} --port ${MONGO_PORT} ${DUMP_DIR}"
mongorestore --drop --host "${MONGO_HOST}" --port "${MONGO_PORT}" "${DUMP_DIR}"

# The shipped seed embeds a `description` (string) field in many `parameter`
# records (initial_pose, target_excavate_pose, ...). The current
# `subtask_excavator_*` implementations iterate over each key and only handle
# numeric types (int32 / int64 / double); the string `description` triggers a
# TypeError and the BT call stalls before sending the goal.
# Strip the field as a post-restore fix-up so task_id=1 etc. work out of the
# box without manual MongoDB editing. Drop this block once the seed (or the
# subtask side) treats `description` properly.
echo "[restore-db] Stripping 'description' from parameter records (workaround for subtask type-handling)"
python3 - "${MONGO_HOST}" "${MONGO_PORT}" <<'PY'
import sys
from pymongo import MongoClient

host, port = sys.argv[1], int(sys.argv[2])
client = MongoClient(host, port)
result = client["rostmsdb"]["parameter"].update_many(
    {"description": {"$exists": True}},
    {"$unset": {"description": ""}},
)
print(f"[restore-db]   modified {result.modified_count} parameter records")
PY

echo "[restore-db] Done."
