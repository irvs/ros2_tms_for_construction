#!/usr/bin/env python3

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
import rclpy
from rclpy.node import Node

import time

import pymongo
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

NODE_NAME = "tms_ur_if_to_db"



class TmsURIfToDbWriter(Node):
    """Convert PoseStamped msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameter('if_file_name', 'if_file_name')

        self.if_file_name = self.get_parameter("if_file_name").get_parameter_value().string_value

        client = pymongo.MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        self.collection = db['parameter_test']

        self.parameter_info = {"model_name":None, "type":None, "x":None, "y":None, "z":None, "qx":None, "qy":None, "qz":None, "qw":None, "record_name":None,"section_id":None , "lavel": None, "related_point_up_main": None, "related_point_up_sub": None, "related_point_down_main":None , "related_point_down_sub":None, "preferred_direction": None}

        self.get_logger().info("DBWriter service is ready")

        try:
            with open(self.if_file_name, 'r', encoding='utf-8') as f:
           # with open('/home/common/output_260217_5_test_2.json', 'r', encoding='utf-8') as f:
                data = json.load(f)

                waypoint_index = self.search_node_link(data)

                for i in range(len(data["taskset_0"][0]["parameters"]["graph"])):
                    self.create_db_data(data , i, waypoint_index)
                    self.write_param()
                    #time.sleep(1)
                self.get_logger().info("All parameter data is written.")
        
        except FileNotFoundError:
            print("File not found.")
        except json.JSONDecodeError:
            print("Incorrect JSON format.")

        self.destroy_node()
        rclpy.shutdown()
    
    def search_node_link(self, data):
        waypoints = data["taskset_0"][0]["parameters"]["graph"]

        id_to_index = {node["id"]: i for i, node in enumerate(waypoints)}
        remain_id_list = {node["id"] for i, node in enumerate(waypoints)}
        dist = {node["id"]: float("inf") for node in waypoints}
    
        starting_point = data["taskset_0"][0]["parameters"]["load"][0]
        dist[starting_point]=0
    
        current_id = starting_point
        remain_id_list.discard(current_id)
        next_ids = waypoints[id_to_index.get(starting_point)]["list"]
        #search relation nodes except sub loads and calculate the number of nodes from the starting point
        for i in range(len(dist)):
            for i in range(len(next_ids)):
                if dist[waypoints[id_to_index.get(current_id)]["list"][i]] == float("inf"):
                    dist[waypoints[id_to_index.get(current_id)]["list"][i]] = dist[current_id] + 1
                    if waypoints[id_to_index.get(current_id)]["list"][i] in data["taskset_0"][0]["parameters"]["main_road"]:
                        next_node = waypoints[id_to_index.get(current_id)]["list"][i]
                        remain_id_list.discard(next_node)
            
            current_id = next_node
            next_ids = waypoints[id_to_index.get(next_node)]["list"]

        remain_id_list = sorted(remain_id_list, key=lambda x: dist[x])

        #search relation nodes sub loads and calculate the number of nodes from the starting point
        for i in range(len(remain_id_list)):
            if len(remain_id_list)>0:
                num = dist[remain_id_list[0]]
                dist[remain_id_list[0]] = [num,1]
                current_id = remain_id_list[0]
                prev_id = remain_id_list[0]
                remain_id_list.remove(prev_id)

                while len(waypoints[id_to_index.get(current_id)]["list"]) == 2:
                    if waypoints[id_to_index.get(current_id)]["list"][0] != prev_id and waypoints[id_to_index.get(current_id)]["list"][0] in data["taskset_0"][0]["parameters"]["sub_road"]:
                        dist[waypoints[id_to_index.get(current_id)]["list"][0]] = [num, dist[current_id][1] + 1]
                        next_id = waypoints[id_to_index.get(current_id)]["list"][0]
                    
                    elif waypoints[id_to_index.get(current_id)]["list"][1] != prev_id and waypoints[id_to_index.get(current_id)]["list"][1] in data["taskset_0"][0]["parameters"]["sub_road"]:
                        dist[waypoints[id_to_index.get(current_id)]["list"][1]] = [num, dist[current_id][1] + 1]
                        next_id = waypoints[id_to_index.get(current_id)]["list"][1]
                    else:
                        break
                    remain_id_list.remove(next_id)
                    prev_id = current_id
                    current_id = next_id  
        #print(dist)

        return dist


    def create_db_data(self, data, num, index):

        self.parameter_info = {"model_name":None, "type":None, "x":None, "y":None, "z":None, "qx":None, "qy":None, "qz":None, "qw":None, "record_name":None,"section_id":None , "lavel": None, "related_point_up_main": None, "related_point_up_sub": None, "related_point_down_main":None , "related_point_down_sub":None, "preferred_direction": None}

        self.parameter_info["type"] = "static"
       # self.parameter_info["id"] = data["taskset_0"][0]["id"]

        vehicles = data["taskset_0"][0]["parameters"]["machinery"]
        vehicle_name = []
        for i in range(len(vehicles)):
            vehicle_name.append(vehicles[i]["name"] + "_" + str(vehicles[i]["params"]))
        self.parameter_info["model_name"] = vehicle_name
        
        waypoints = data["taskset_0"][0]["parameters"]["graph"]

        section_id = waypoints[num]["id"]

        self.parameter_info["section_id"] = section_id
        self.parameter_info["record_name"] = section_id
        self.parameter_info["x"] = waypoints[num]["position"]["x"]
        self.parameter_info["y"] = waypoints[num]["position"]["y"]
        self.parameter_info["z"] = waypoints[num]["position"]["z"]
        related_points = waypoints[num]["list"]

        self.starting_point = data["taskset_0"][0]["parameters"]["load"]
        
        for i in range(len(related_points)):
            this_distance = index.get(section_id)
            relate_point_distance = index.get(related_points[i])
            if type(this_distance)==list and type(relate_point_distance)==int:
                this_distance = this_distance[0]
            elif type(this_distance)==int and type(relate_point_distance)==list:
                relate_point_distance = relate_point_distance[0]
            elif type(this_distance)==list and type(relate_point_distance)==list:
                relate_point_distance = relate_point_distance[1]
                this_distance = this_distance[1]
            if this_distance < relate_point_distance and related_points[i] in data["taskset_0"][0]["parameters"]["main_road"]:
                self.parameter_info["related_point_up_main"] = related_points[i]
            elif this_distance < relate_point_distance and related_points[i] in data["taskset_0"][0]["parameters"]["sub_road"]:
                self.parameter_info["related_point_up_sub"] = related_points[i]
            elif this_distance > relate_point_distance and related_points[i] in data["taskset_0"][0]["parameters"]["main_road"]:
                self.parameter_info["related_point_down_main"] = related_points[i]
            elif this_distance > relate_point_distance and related_points[i] in data["taskset_0"][0]["parameters"]["sub_road"]:
                self.parameter_info["related_point_down_sub"] = related_points[i]

        if section_id in data["taskset_0"][0]["parameters"]["main_road"]:
            self.parameter_info["lavel"] = "main"
        elif section_id in data["taskset_0"][0]["parameters"]["sub_road"]:
            self.parameter_info["lavel"] = "sub"

        self.parameter_info["preferred_direction"] = "up"

        return

    def write_param(self):
        self.collection.insert_one(self.parameter_info)
        self.get_logger().info(f"Completed Inseting parameter data into the rostmsdb database !")


def main(args=None):
    rclpy.init(args=args)
    tms_ur_db_writer = TmsURIfToDbWriter()
    # rclpy.spin(tms_ur_db_writer)
    # tms_ur_db_writer.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()