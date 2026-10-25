#include "tms_ts_primitive/Excavator/lib/excavator_pose_converter.hpp"

ExcavatorPoseConverter::ExcavatorPoseConverter()
{
}

ExcavatorPoseConverter::~ExcavatorPoseConverter()
{
}

void ExcavatorPoseConverter::convertToXYZQuaternion(const double x, const double y, const double z, const double theta_w, Pose& result_pose)
{
  // 位置は入力と同じ（bucket_edgeの位置）
  result_pose.x = x;
  result_pose.y = y;
  result_pose.z = z;

  // theta_wはバケット底面の水平面に対する傾き（刃先が下がる向きが正）
  // フレームのx軸は底面に対して kBucketFrameOffset だけずれているので、その分を加える
  double pitch = theta_w + kBucketFrameOffset;

  // 姿勢の計算
  double roll = 0.0;         // ロール角は0と仮定
  double yaw = atan2(y, x);  // ヨー角は旋回角（swing_joint）として計算

  // オイラー角からクォータニオンに変換
  eulerToQuaternion(roll, pitch, yaw, result_pose.qx, result_pose.qy, result_pose.qz, result_pose.qw);
}

void ExcavatorPoseConverter::eulerToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  qw = cy * cp * cr + sy * sp * sr;
  qx = cy * cp * sr - sy * sp * cr;
  qy = sy * cp * sr + cy * sp * cr;
  qz = sy * cp * cr - cy * sp * sr;
}