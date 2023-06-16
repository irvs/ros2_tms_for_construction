import json
import math
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import TransformStamped

import tkinter as tk

import quaternion

NODE_NAME: str = "ground_tf_broadcaster"
WINDOW_TITLE: str = "Ground TF Broadcaster"
WINDOW_WIDTH: int = 500
WINDOW_HEIGHT: int = 440
WINDOW_BG_COLOR: str = "white"


class GroundTFBroadcaster(tk.Frame):
    """GUI for broadcasting TF of ground."""

    def __init__(self, master):
        super().__init__(master)
        self.pack()

        self.translation_x = tk.Scale(
            self,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=20,
            digits=3,
            label="Translation X",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.translation_y = tk.Scale(
            self,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=20,
            digits=3,
            label="Translation Y",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        self.translation_z = tk.Scale(
            self,
            from_=-100,
            to=100,
            orient=tk.HORIZONTAL,
            resolution=0.1,
            length=500,
            tickinterval=20,
            digits=3,
            label="Translation Z",
            background="white",
            troughcolor="blue",
            font=("", 15),
        )
        # self.rotation_x = tk.Scale(
        #     self,
        #     from_=-180,
        #     to=180,
        #     orient=tk.HORIZONTAL,
        #     resolution=1,
        #     length=500,
        #     tickinterval=30,
        #     digits=3,
        #     label="Rotation X",
        # )
        # self.rotation_y = tk.Scale(
        #     self,
        #     from_=-180,
        #     to=180,
        #     orient=tk.HORIZONTAL,
        #     resolution=1,
        #     length=500,
        #     tickinterval=30,
        #     digits=3,
        #     label="Rotation Y",
        # )
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
        # self.rotation_x.pack()
        # self.rotation_y.pack()
        self.rotation_z.pack()
        self.save_button.pack()

        rclpy.init()
        self.node = Node(NODE_NAME)

        # Declare parameters
        self.node.declare_parameter("to_frame", "world")
        self.node.declare_parameter("from_frame", "laser_link")
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
            ground_config = json.load(f)["ground"]
        self.translation_x.set(ground_config["translation"]["x"])
        self.translation_y.set(ground_config["translation"]["y"])
        self.translation_z.set(ground_config["translation"]["z"])
        # self.rotation_x.set(ground_config["rotation"]["x"])
        # self.rotation_y.set(ground_config["rotation"]["y"])
        self.rotation_z.set(ground_config["rotation"]["z"])

        # Initialize TF
        self.t = TransformStamped()
        self.t.header.frame_id = self.to_frame
        self.t.child_frame_id = self.from_frame

        self.tf_broadcaster = TransformBroadcaster(node=self.node)

        self.timeEvent()

    def timeEvent(self) -> None:
        """Broadcast TF of ground."""
        self.t.header.stamp = self.node.get_clock().now().to_msg()

        self.t.transform.translation.x = float(self.translation_x.get())
        self.t.transform.translation.y = float(self.translation_y.get())
        self.t.transform.translation.z = float(self.translation_z.get())

        q = quaternion.from_euler_angles(
            # math.radians(float(self.rotation_x.get())),
            # math.radians(float(self.rotation_y.get())),
            0.0,
            0.0,
            math.radians(float(self.rotation_z.get())),
        )
        self.t.transform.rotation.x = q.x
        self.t.transform.rotation.y = q.y
        self.t.transform.rotation.z = q.z
        self.t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(self.t)
        self.after(1000, self.timeEvent)

    def save(self) -> None:
        """Save the current value of translation and rotation."""
        ground_config = {
            "translation": {
                "x": self.translation_x.get(),
                "y": self.translation_y.get(),
                "z": self.translation_z.get(),
            },
            "rotation": {
                "x": 0,
                "y": 0,
                "z": self.rotation_z.get(),
            },
        }
        with open(self.config_file, "r+") as f:
            tf_config = json.load(f)
            tf_config["ground"] = ground_config
            f.seek(0)
            json.dump(tf_config, f, indent=4)
            f.truncate()


def main():
    window = tk.Tk()
    window.title(WINDOW_TITLE)
    window.configure(bg=WINDOW_BG_COLOR)
    window.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
    app = GroundTFBroadcaster(window)
    app.mainloop()


if __name__ == "__main__":
    main()
