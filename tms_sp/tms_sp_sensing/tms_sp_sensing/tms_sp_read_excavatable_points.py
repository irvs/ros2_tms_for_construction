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

"""
Sample script that reads the "excavatable_points" candidates
(x, y, z, theta_w) stored under rostmsdb.parameter for a given
record_name, and prints them out.

Target document shape (rostmsdb.parameter):
{
  "record_name": "target_excavate_pose_2",
  "excavatable_points": [
    {"x": -1.5, "y": -6.5, "z": -2.5, "theta_w": 1},
    ...
  ]
}
"""

from pymongo import MongoClient

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

RECORD_NAME = 'target_excavate_pose_2'


def get_excavatable_points(record_name: str) -> list:
    """
    Read the excavatable_points field from rostmsdb.parameter
    for the given record_name.

    Parameters
    ----------
    record_name : str
        Value of the "record_name" field to look up.

    Returns
    -------
    list
        List of {"x", "y", "z", "theta_w"} dicts. Empty list if
        the record or the field was not found.
    """
    client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
    db = client['rostmsdb']
    collection = db['parameter']

    query = {"record_name": record_name}
    parameter_info = collection.find_one(query)

    if parameter_info is None:
        print(f"No document found for record_name='{record_name}'")
        return []

    return parameter_info.get('excavatable_points', [])


def main():
    excavatable_points = get_excavatable_points(RECORD_NAME)

    print(f"record_name: {RECORD_NAME}")
    print(f"number of excavatable_points: {len(excavatable_points)}")

    for i, point in enumerate(excavatable_points):
        x = point.get('x')
        y = point.get('y')
        z = point.get('z')
        theta_w = point.get('theta_w')
        print(f"[{i}] x={x}, y={y}, z={z}, theta_w={theta_w}")


if __name__ == '__main__':
    main()
