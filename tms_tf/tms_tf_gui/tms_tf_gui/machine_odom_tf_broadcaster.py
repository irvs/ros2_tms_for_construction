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
import math
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

import tkinter as tk

import numpy as np
import quaternion


NODE_NAME: str = "machine_odom_tf_broadcaster"
WINDOW_TITLE: str = "Machine Odom TF Broadcaster"
WINDOW_WIDTH: int = 500
WINDOW_HEIGHT: int = 650
WINDOW_BG_COLOR: str = "white"


class OdomTFBroadcaster(tk.Frame):
    """GUI for broadcasting TF of odom."""

    def __init__(self, master):
        super().__init__(master)
        self.pack()

        self.translation_x = tk.Scale(
            self,
            from_=-20,
            to=20,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=5,
            digits=3,
            label="Pose X",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.translation_y = tk.Scale(
            self,
            from_=-20,
            to=20,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=5,
            digits=3,
            label="Pose Y",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.translation_z = tk.Scale(
            self,
            from_=-20,
            to=20,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=5,
            digits=3,
            label="Pose Z",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.rotation_x = tk.Scale(
            self,
            from_=-180,
            to=180,
            orient=tk.HORIZONTAL,
            resolution=1,
            length=500,
            tickinterval=30,
            digits=3,
            label="Rotation X",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.rotation_y = tk.Scale(
            self,
            from_=-180,
            to=180,
            orient=tk.HORIZONTAL,
            resolution=1,
            length=500,
            tickinterval=30,
            digits=3,
            label="Rotation Y",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.rotation_z = tk.Scale(
            self,
            from_=-180,
            to=180,
            orient=tk.HORIZONTAL,
            resolution=1,
            length=500,
            tickinterval=30,
            digits=3,
            label="Rotation Z",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.save_button = tk.Button(
            self,
            text="Save",
            command=self.save,
            background="white",
        )

        self.translation_x.pack()
        self.translation_y.pack()
        self.translation_z.pack()
        self.rotation_x.pack()
        self.rotation_y.pack()
        self.rotation_z.pack()
        self.save_button.pack()

        rclpy.init()
        self.node = Node(NODE_NAME)

        # Declare parameters
        self.node.declare_parameter("to_frame", "world")
        self.node.declare_parameter("from_frame", "map")
        self.node.declare_parameter("config_file", "tf_config.json")

        # Get parameters
        self.to_frame = (
            self.node.get_parameter("to_frame").get_parameter_value().string_value
        )
        self.from_frame = (
            self.node.get_parameter("from_frame").get_parameter_value().string_value
        )
        self.config_file = (
            self.node.get_parameter("config_file").get_parameter_value().string_value
        )

        # Set default value
        with open(self.config_file) as f:
            odom_config = json.load(f)["machine_odom"]
        self.translation_x.set(odom_config["translation"]["x"])
        self.translation_y.set(odom_config["translation"]["y"])
        self.translation_z.set(odom_config["translation"]["z"])
        self.rotation_x.set(odom_config["rotation"]["x"])
        self.rotation_y.set(odom_config["rotation"]["y"])
        self.rotation_z.set(odom_config["rotation"]["z"])

        # Initialize TF
        self.t = TransformStamped()
        self.t.header.frame_id = self.to_frame
        self.t.child_frame_id = self.from_frame

        self.tf_broadcaster = TransformBroadcaster(node=self.node)

        self.timeEvent()

    def timeEvent(self) -> None:
        """Broadcast TF of odom."""
        self.t.header.stamp = self.node.get_clock().now().to_msg()

        self.t.transform.translation.x = float(self.translation_x.get())
        self.t.transform.translation.y = float(self.translation_y.get())
        self.t.transform.translation.z = float(self.translation_z.get())

        # q = self.euler_to_quaternion(
        #     math.radians(float(self.rotation_x.get())),
        #     math.radians(float(self.rotation_y.get())),
        #     math.radians(float(self.rotation_z.get())),
        # )
        q = quaternion.from_euler_angles(
            math.radians(float(self.rotation_x.get())),
            math.radians(float(self.rotation_y.get())),
            math.radians(float(self.rotation_z.get())),
        )

        q = q.normalized()

        self.t.transform.rotation.x = q.x
        self.t.transform.rotation.y = q.y
        self.t.transform.rotation.z = q.z
        self.t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(self.t)
        self.after(1000, self.timeEvent)

    def save(self) -> None:
        """Save the current value of translation and rotation."""
        odom_config = {
            "translation": {
                "x": self.translation_x.get(),
                "y": self.translation_y.get(),
                "z": self.translation_z.get(),
            },
            "rotation": {
                "x": self.rotation_x.get(),
                "y": self.rotation_y.get(),
                "z": self.rotation_z.get(),
            },
        }
        with open(self.config_file, "r+") as f:
            tf_config = json.load(f)
            tf_config["machine_odom"] = odom_config
            f.seek(0)
            json.dump(tf_config, f, indent=4)
            f.truncate()

    # def euler_to_quaternion(self, roll, pitch, yaw) -> np.quaternion:
    #     cr = math.cos(roll / 2)
    #     sr = math.sin(roll / 2)
    #     cp = math.cos(pitch / 2)
    #     sp = math.sin(pitch / 2)
    #     cy = math.cos(yaw / 2)
    #     sy = math.sin(yaw / 2)

    #     q = np.quaternion(
    #         cr * cp * cy + sr * sp * sy,
    #         sr * cp * cy - cr * sp * sy,
    #         cr * sp * cy + sr * cp * sy,
    #         cr * cp * sy - sr * sp * cy,
    #     )

    #     return q


def main():
    window = tk.Tk()
    window.title(WINDOW_TITLE)
    window.configure(bg=WINDOW_BG_COLOR)
    window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
    app = OdomTFBroadcaster(window)
    app.mainloop()


if __name__ == "__main__":
    main()
