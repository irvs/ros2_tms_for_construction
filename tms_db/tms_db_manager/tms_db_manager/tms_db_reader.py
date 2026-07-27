# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import json
import pymongo

import rclpy
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped



class TmsDbReader(Node):
    """Read data from MongoDB."""

    def __init__(self):
        super().__init__("tms_db_reader")

        # Declare parameters
        self.declare_parameter("db_host", '127.0.0.1')
        self.declare_parameter("db_port", 27017)

        # Get parameters
        self.db_host: str = (
            self.get_parameter("db_host").get_parameter_value().string_value
        )
        self.db_port: int = (
            self.get_parameter("db_port").get_parameter_value().integer_value
        )

        self.db: pymongo.database.Database = db_util.connect_db(
            "rostmsdb", self.db_host, self.db_port
        )
        self.srv = self.create_service(
            TmsdbGetData, "tms_db_reader", self.db_reader_srv_callback
        )

    def db_reader_srv_callback(self, request, response):
        """
        Respond to requests from client nodes.

        Parameters
        ----------
        request
            Request from client node.
        response
            Response to client node.

        Returns
        -------
        response
            Response to client node.
        """
        collection: pymongo.collection.Collection = self.db[request.type]

        if request.latest_only and not (request.param_type == "path_plan" or request.param_type == "joint_plan"):
            latest_data: dict = self.get_latest_data(request, collection)
            if latest_data == None:
                return response
            response.tmsdbs.append(self.allocate_tmsdb(latest_data))
            return response
        
        elif request.latest_only and request.param_type == "path_plan":
            plan_data: dict = self.get_path_plan_data(request, collection)
            if plan_data is None:
                self.get_logger().info("get no plan")
                return response

            path_plan = self.plan_to_path(plan_data)
            response.tmsdbs.append(self.path_plan_tmsdb(path_plan))
            self.get_logger().info("return plan")
            return response
        
        elif request.latest_only and request.param_type == "joint_plan":
            #self.get_logger().info("obtain joint plan request")
            for i in range(len(request.recordnames) - 1):
                plan_data: dict = self.get_joint_plan_data(request, collection, i + 1)
                if plan_data is None:
                    self.get_logger().info("get no plan")
                    return response

                self.get_logger().info("plan_data")
                joint_path = self.plan_to_joint(plan_data)
                self.get_logger().info("joint_path")
                response.tmsdbs.append(self.joint_plan_tmsdb(joint_path))

            self.get_logger().info("return plan")
            return response

        else:
            all_data: pymongo.cursor.Cursor = self.get_all_data(request, collection)
            for data in all_data:
                msg: Tmsdb = self.allocate_tmsdb(data)
                response.tmsdbs.append(msg)
            return response

    def get_latest_data(
        self, request, collection: pymongo.collection.Collection
    ) -> dict:
        """
        Get latest data only.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        dict
            Requested latest data.
        """
        if request.name != "":
            latest_data: dict = collection.find_one(
                {"id": request.id, "name": request.name},
                sort=[("time", pymongo.DESCENDING)],
            )
        else:
            latest_data: dict = collection.find_one(
                {"id": request.id}, sort=[("time", pymongo.DESCENDING)]
            )

        return latest_data

    def get_all_data(self, request, collection) -> pymongo.cursor.Cursor:
        """
        Get all data.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        pymongo.cursor.Cursor
            Requested all data.
        """
        if request.name != "":
            all_data: pymongo.cursor.Cursor = collection.find(
                {"id": request.id, "name": request.name}
            ).sort([("time", pymongo.ASCENDING)])
        else:
            all_data: pymongo.cursor.Cursor = collection.find({"id": request.id}).sort(
                [("time", pymongo.ASCENDING)]
            )
        return all_data
    
    # def get_plan_data(self, request, collection) -> dict:
    #     if request.name != "":
    #         plan_data = collection.find_one(
    #             {"record_name": request.recordnames[0], "name": request.name},
    #             sort=[("time", pymongo.DESCENDING)],
    #         )
    #     else:
    #         plan_data = collection.find_one(
    #             {"id": request.id},
    #             sort=[("time", pymongo.DESCENDING)],
    #         )
    #     return plan_data
    
    def get_path_plan_data(self, request, collection) -> dict:
        # name が空でない場合
        if request.name != "":
            plan_data = collection.find_one(
                #{"record_name": request.recordnames[0], "model_name": request.name}
                {"type": "path_plan", "model_name": request.name}
            )
        # name が空の場合
        else:
            plan_data = collection.find_one(
                {"id": request.id}
            )
        return plan_data
    
    def get_joint_plan_data(self, request, collection, num) -> dict:
        # name が空でない場合
        if request.name != "":
            plan_data = collection.find_one(
                {"record_name": request.recordnames[num], "model_name": request.name}
                # {"type": "joint_plan", "model_name": request.name}
            )
        # name が空の場合
        else:
            plan_data = collection.find_one(
                {"id": request.id}
            )
        return plan_data
    
            
    def get_plan_data(self, request, collection, num) -> dict:

        if request.name != "":
            self.get_logger().info(f"record_name: {request.recordnames[num]}, model_name: {request.name}")
            return collection.find_one({"record_name": request.recordnames[num], "model_name": request.name})
        else:
            return collection.find_one({"id": request.id}, sort=[("time", pymongo.DESCENDING)])


    def allocate_tmsdb(self, data: dict) -> Tmsdb:
        """
        Allocate dictionary data to Tmsdb msg.

        Parameters
        ----------
        dict : data
            Dictionary data.

        Returns
        -------
        Tmsdb
            Tmsdb msg data.
        """
        tmsdb = Tmsdb()
        tmsdb.time = data["time"]
        tmsdb.type = data["type"]
        tmsdb.id = data["id"]
        tmsdb.name = data["name"]
        tmsdb.msg = json.dumps(data["msg"])
        return tmsdb
    
    def path_plan_tmsdb(self, data: dict) -> Tmsdb:
        tmsdb = Tmsdb()
        tmsdb.pathplan = data
        return tmsdb
    
    def joint_plan_tmsdb(self, data: dict) -> Tmsdb:
        tmsdb = Tmsdb()
        tmsdb.jointplan = data
        return tmsdb
    

    def plan_to_path(self, data: dict, frame_id: str = "map") -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        xs = data["x"]
        ys = data["y"]
        zs = data["z"]
        qxs = data["qx"]
        qys = data["qy"]
        qzs = data["qz"]
        qws = data["qw"]
            

        for i in range(len(xs)):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path.header.stamp
            pose_stamped.header.frame_id = frame_id

            pose_stamped.pose.position.x = xs[i]
            pose_stamped.pose.position.y = ys[i]
            pose_stamped.pose.position.z = zs[i]

            pose_stamped.pose.orientation.x = qxs[i]
            pose_stamped.pose.orientation.y = qys[i]
            pose_stamped.pose.orientation.z = qzs[i]
            pose_stamped.pose.orientation.w = qws[i]

            path.poses.append(pose_stamped)
        
        return path

    def plan_to_joint(self, data: dict, frame_id: str = "map") -> JointTrajectory:
        jointplan = JointTrajectory()
        traj = JointTrajectory()
        traj.joint_names = traj.joint_names = data["plan"]["joint_trajectory"]["joint_names"]
    

        for p in data["plan"]["joint_trajectory"]["points"]:
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in p["positions"]]
            point.velocities = [float(v) for v in p["velocities"]]
            point.accelerations = [float(v) for v in p["accelerations"]]
            point.time_from_start.sec = p["time_from_start"]["sec"]
            point.time_from_start.nanosec = p["time_from_start"]["nanosec"]
            traj.points.append(point)

        return traj


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader = TmsDbReader()
    rclpy.spin(tms_db_reader)

    tms_db_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
