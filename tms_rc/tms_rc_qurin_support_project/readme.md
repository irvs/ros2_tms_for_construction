# tms_rc_qurin_support
Qurin　屋内見守りロボット

## Scripts
### pozyx

pozyx creatorからデータを取得，
EKFノードにOdometryトピックを発行．

#### publish
* odometry/pozyx (nav_msgs/msg/Odometry)

### guidebot_odometry
Robot端末からオドメトリを取得，
TF(odom→base_footprint)，Odometryトピックを発行．

#### publish
* (nav_msgs/msg/Odometry)
* TF (odom→base_footprint)

## Launch
* 
* 

