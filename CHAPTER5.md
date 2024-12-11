### 5. Try running the task schedular with OperaSim-PhysX

本章では国立研究開発法人土木研究所が開発を進めるOPERAの一部であるシミュレータOperaSim-PhysXとROS2-TMS for Constructionを接続し、ROs2-TMS for ConstructionからOperaSima-PhysX上の建設機械を操作する方法についてご紹介します。

まずはじめに、下記の点に注意しつつ[OperaSim-PhysXの公式ページ](https://github.com/pwri-opera/OperaSim-PhysX)のReadMeに記載されている手順にそってwindows PC上で環境構築を行ってください。

・OperaSim-PhysXを起動するPCとDockerを立ち上げるホストPCは同一ネットワーク上にあることを確認してください。

・Docker上のROS2ノードとOperaSim-PhysXを接続する際には、OperaSim-PhysXの Robotics → ROS Settingsの設定を以下の通り設定してください

![](docs/SimParam_Setting.png)


OperaSim-PhysX とUbuntu 22.04 PCの接続ができたら、以下のコマンドを実行してください。
コマンドは実行するタスクごとに異なるので、該当する項に沿って実行してください。
なお、[実行したいタスクのtask_id]は以下に示す例のように実行したいDB上のタスクデータのtask_idの値に置き換えてください。

```
例　([実行したいタスクのtask_id]=1の場合)：　ros2 launch tms_ts_launch tms_ts_construction.launch.py　task_id:=1
```

現在、データベースに用意されているタスクの詳細は以下の表のとおりです。

![](docs/task_data.png)


#### Packages for operating OPERA-compatible machineries on the OperaSim-PhysX　(task_id: 1, 2, 3)

```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py

# Open the 2nd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py　task_id:=[実行したいタスクのtask_id]
```

#### Packages for operating OPERA-compatible ZX200 on the OperaSim-PhysX using MoveIt! (task_id: 4, 5)
```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py

# Open the 2nd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch zx200_unity zx200_standby.launch.py

# Open the 3rd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py

# Open the 4th terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py　task_id:=[実行したいタスクのtask_id]
```
#### Packages for operating OPOERA-compatible IC120 on the OperaSim-PhysX using Nav2! (task_id: 6, 7) 
```
# Open the 1st terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ros_tcp_endpoint endpoint.py

# Open the 2nd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch ic120_unity ic120_standby_ekf.launch.py

# Open the 3rd terminal
cd ~/ros2-tms-for-construction_ws && source install/setup.bash
ros2 launch tms_ts_launch tms_ts_construction.launch.py　task_id:=[実行したいタスクのtask_id]
```


