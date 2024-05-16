#!/bin/bash

# build the rostmsdb database on MongoDB
mongod --bind_ip_all &

while ! mongo --eval "print(\"wait for mongodb to start\")"
do
  sleep 1
done

mongorestore $OVERLAY_WS/src/ros2_tms_for_construction/demo/dump

fg %1
