import cv2
from datetime import datetime
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tms_msg_db.msg import TmsdbGridFS


NODE_NAME = 'tms_ss_terrain_static_dem'
DATA_ID   = 3030
DATA_TYPE = 'dem'

class TmsSsTerrainStaticDem(Node):
    """Create DEM from PointCloud2 and send the file name to tms_db_writer_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('filename_dem', 'filename_dem.npy')
        self.declare_parameter('fill_nan_type', '')  # '' or 'avg' or 'median'
        self.declare_parameter('resolution', 0.1)  # [m]

        # Get parameters
        self.filename_dem: str  = self.get_parameter('filename_dem').get_parameter_value().string_value
        self.fill_nan_type: str = self.get_parameter('fill_nan_type').get_parameter_value().string_value
        self.resolution: float  = self.get_parameter('resolution').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(TmsdbGridFS, 'tms_db_gridfs_data', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            'tms_sd_terrain_static',
            self.send_dem_to_db_writer_gridfs,
            10,
        )

    def send_dem_to_db_writer_gridfs(self, msg: PointCloud2) -> None:
        """
        Send topics to tms_db_writer_gridfs (Write the received PointCloud2 data to DB as a npy file of DEM).

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        self.msg = msg
        self.pointcloud2_dem_handler()

        tmsdb_gridfs_msg          = TmsdbGridFS()
        tmsdb_gridfs_msg.time     = datetime.now().isoformat()
        tmsdb_gridfs_msg.type     = DATA_TYPE
        tmsdb_gridfs_msg.id       = DATA_ID
        tmsdb_gridfs_msg.filename = self.filename_dem

        self.publisher_.publish(tmsdb_gridfs_msg)

    def pointcloud2_dem_handler(self) -> None:
        """
        Convert PointCloud2 msg to DEM and save it as a npy file.
        """
        self.get_logger().info("Creating a DEM ...")
        dem: np.ndarray = self.convert_pointcloud2_to_dem()
        np.save(self.filename_dem, dem)

    def convert_pointcloud2_to_dem(self) -> np.ndarray:
        """
        Convert PointCloud2 msg to DEM.
        
        Returns
        -------
        dem : np.ndarray
            DEM.
        """
        field_names = [field.name for field in self.msg.fields]

        points = []
        for x, y, z, _, _, _ in list(point_cloud2.read_points(self.msg, skip_nans=True, field_names=field_names)):
            points.append([x, y, z])
        cloud = np.array(points)

        # # Generate DEM
        # x_min, x_max = np.min(cloud[:, 0]), np.max(cloud[:, 0])
        # y_min, y_max = np.min(cloud[:, 1]), np.max(cloud[:, 1])

        # # Calculate the num of cell
        # x_num = int(np.ceil((x_max - x_min) / self.resolution))
        # y_num = int(np.ceil((y_max - y_min) / self.resolution))

        # # Calculate the average height in each cell
        # dem = np.empty((y_num, x_num), dtype=np.float32)
        # dem.fill(np.nan)
        # for x, y, z in cloud:
        #     ix = int((x - x_min) / self.resolution)
        #     iy = int((y - y_min) / self.resolution)
        #     if 0 <= ix and ix < x_num and 0 <= iy and iy < y_num:
        #         if np.isnan(dem[iy, ix]):
        #             dem[iy, ix] = z
        #         else:
        #             dem[iy, ix] = (dem[iy, ix] + z) / 2

        # if (self.fill_nan_type == 'avg'):
        #     # Fill NaN with the average value of surrounding cells
        #     for i in range(1, y_num):
        #         for j in range(1, x_num):
        #             if np.isnan(dem[i, j]):
        #                 surrounding_cells = dem[i-1:i+2, j-1:j+2]
        #                 dem[i, j] = np.nanmean(surrounding_cells)
        # elif (self.fill_nan_type == 'median'):
        #     # Fill NaN with median value
        #     dem[np.isnan(dem)] = np.nanmedian(dem)

        # dem_normalized = cv2.normalize(
        #     dem, None, -128, 127, cv2.NORM_MINMAX, cv2.CV_8UC1)

        dem_normalized = self.generate_dem(cloud)

        return dem_normalized
    
    def generate_dem(self, cloud: np.ndarray) -> np.ndarray:
        """
        Generate DEM from point cloud.

        Parameters
        ----------
        cloud : np.ndarray
            Point cloud.

        Returns
        -------
        dem : np.ndarray
            DEM.
        """
        x_min, x_max = np.min(cloud[:, 0]), np.max(cloud[:, 0])
        y_min, y_max = np.min(cloud[:, 1]), np.max(cloud[:, 1])

        # Calculate the num of cell
        x_num = int(np.ceil((x_max - x_min) / self.resolution))
        y_num = int(np.ceil((y_max - y_min) / self.resolution))

        # Calculate the average height in each cell
        dem = np.empty((y_num, x_num), dtype=np.float32)
        dem.fill(np.nan)
        for x, y, z in cloud:
            ix = int((x - x_min) / self.resolution)
            iy = int((y - y_min) / self.resolution)
            if 0 <= ix and ix < x_num and 0 <= iy and iy < y_num:
                if np.isnan(dem[iy, ix]):
                    dem[iy, ix] = z
                else:
                    dem[iy, ix] = (dem[iy, ix] + z) / 2

        if (self.fill_nan_type == 'avg'):
            # Fill NaN with the average value of surrounding cells
            for i in range(1, y_num):
                for j in range(1, x_num):
                    if np.isnan(dem[i, j]):
                        surrounding_cells = dem[i-1:i+2, j-1:j+2]
                        dem[i, j] = np.nanmean(surrounding_cells)
        elif (self.fill_nan_type == 'median'):
            # Fill NaN with median value
            dem[np.isnan(dem)] = np.nanmedian(dem)

        dem_normalized = cv2.normalize(
            dem, None, -128, 127, cv2.NORM_MINMAX, cv2.CV_8UC1)

        return dem_normalized
    

def main(args=None):
    rclpy.init(args=args)

    tms_ss_terrain_static_dem = TmsSsTerrainStaticDem()
    rclpy.spin_once(tms_ss_terrain_static_dem)

    tms_ss_terrain_static_dem.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()