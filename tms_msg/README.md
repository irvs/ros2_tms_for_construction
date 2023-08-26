# TMS Messages

Actinos, Messages and Services used in ROS2-TMS-FOR-CONSTRUCTION.

## Actions

### TmsdbGridFS

TmsdbGridFS is used by [tms_db_reader_gridfs](../tms_db/tms_db_manager/tms_db_manager/tms_db_reader_gridfs.py) to get file from TMS database.

```
string type
int32  id
string filename
bool   latest_only
---
bool   result
string msg
sensor_msgs/PointCloud2 pointcloud2
---
bool   result
string msg
sensor_msgs/PointCloud2 pointcloud2
```

## Messages

### ColoredMesh

ColoredMesh is used by ColoredMeshSrv.

```
shape_msgs/MeshTriangle[] triangles
geometry_msgs/Point[]     vertices
std_msgs/ColorRGBA[]      vertex_colors
geometry_msgs/Vector3[]   vertex_normals

```

### Tmsdb

Tmsdb is subscribed by [tms_db_writer](../tms_db/tms_db_manager/tms_db_manager/tms_db_writer.py).

```
string time       # Time this msg was created.
string type       # Data type (ex. sensor, machine).
int32  id         # Data id.
string name       # Name (ex. sensor name, machine name).
bool   is_insert  # Whether inserting data or updating
string msg        # JSON msg converted from ROS msg.
```

### TmsdbGridFS

TmsdbGridFS is subscribed by [tms_db_writer_gridfs](../tms_db/tms_db_manager/tms_db_manager/tms_db_writer_gridfs.py).

```
string time       # Time this msg was created.
string type       # Data type (ex. sensor, machine).
int32  id         # Data id 
string filename   # File name of the file to be stored in the database.
```

## Services

### ColoredMeshSrv

ColoredMeshSrv is used by [tms_ur_construction_terrain_static](../tms_ur/tms_ur_construction/tms_ur_construction/tms_ur_construction_terrain_static.py) to get colored mesh data of static terrain.

```
string type
---
tms_msg_db/ColoredMesh colored_mesh
```

### TerrainStaticSrv

TerrainStaticSrv is used by [tms_ur_construction_terrain_mesh](../tms_ur/tms_ur_construction/tms_ur_construction/tms_ur_construction_terrain_mesh.py) to get point cloud data of static terrain.

```
string type
---
sensor_msgs/PointCloud2 pointcloud2
```

### TmsdbGetData

TmsdbGetData is used by [tms_db_reader](../tms_db/tms_db_manager/tms_db_manager/tms_db_reader.py) to get data from TMS database.

```
string type         # Data type (ex. sensor, machine).
int32  id           # Data id.
bool   latest_only  # Whether getting latest data or all stored data.
string name         # Name (ex. sensor name, machine name).
---
tms_msg_db/Tmsdb[]  tmsdbs
```

### TmsdbGridFSGetData

TmsdbGetData is used by [tms_db_reader_gridfs](../tms_db/tms_db_manager/tms_db_manager/tms_db_reader_gridfs.py) to get file from TMS database.

```
string type      # Data type (ex. sensor, machine).
int32  id        # Data id.
string filename  # File name of the file stored in TMS database.
---
bool   result    # Whether the file was successfully got or not.
```
