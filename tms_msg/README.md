# TMS Messages

Messages and Services used in ROS2-TMS-FOR-CONSTRUCTION.

## Messages

### Tmsdb

Tmsdb is subscribed by [tms_db_writer](https://github.com/irvs/ros2_tms_for_construction/blob/main/tms_db/tms_db_manager/tms_db_manager/tms_db_writer.py).

```
string time       # Time this msg was created.
string type       # Data type (ex. sensor, machine).
int32  id         # Data id.
string name
bool   is_insert  # Whether inserting data or updating
string msg        # JSON msg converted from ROS msg.
```

### TmsdbGridFS

TmsdbGridFS is subscribed by [tms_db_writer_gridfs](https://github.com/irvs/ros2_tms_for_construction/blob/main/tms_db/tms_db_manager/tms_db_manager/tms_db_writer_gridfs.py).

```
string time       # Time this msg was created.
string type       # Data type (ex. sensor, machine).
int32  id         # Data id 
string filename   # File name of the file to be stored in the database.
```

## Services

### TmsdbGetData

TmsdbGetData is used by [tms_db_reader](https://github.com/irvs/ros2_tms_for_construction/blob/main/tms_db/tms_db_manager/tms_db_manager/tms_db_reader.py) to get data from TMS database.

```
string type         # Data type (ex. sensor, machine).
int32  id           # Data id.
bool   latest_only  # Whether getting latest data or all stored data.
---
tms_msg_db/Tmsdb[]  tmsdbs
```

### TmsdbGridFSGetData

TmsdbGetData is used by [tms_db_reader_gridfs](https://github.com/irvs/ros2_tms_for_construction/blob/main/tms_db/tms_db_manager/tms_db_manager/tms_db_reader_gridfs.py) to get file from TMS database.

```
string type      # Data type (ex. sensor, machine).
int32  id        # Data id.
string filename  # File name of the file stored in TMS database.
---
bool   result    # Whether the file was successfully got or not.
```
