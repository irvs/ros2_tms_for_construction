#ifndef EXCAVATOR_POSE_CONVERTER_HPP
#define EXCAVATOR_POSE_CONVERTER_HPP

#include <cmath>

struct Pose
{
  double x, y, z;
  double qx, qy, qz, qw;  // quaternion
};

class ExcavatorPoseConverter
{
public:
  ExcavatorPoseConverter();
  ~ExcavatorPoseConverter();

  /* input: position of bucket_edge, バケット底面の水平面に対する傾き theta_w [rad]
   *   theta_w = 0   : 底面が水平で刃先が前方(車体と反対側)を向く
   *   theta_w > 0   : 刃先が下がる向き(掘削時に刃先を地面へ向ける方向)
   *
   * theta_w と刃先向きの対応表（底面のかかと→刃先方向を水平面から測った角度。幾何計算による目安）
   *   theta_w [rad] | [deg] | 刃先の向き
   *   --------------+-------+--------------------------------------------------
   *        -0.52    |  -30  | 刃先が30deg上向き（ダンプ側に開いた状態、土の放出）
   *        -0.26    |  -15  | 刃先がやや上向き
   *         0.00    |    0  | 底面が水平、刃先は前方
   *         0.26    |   15  | 刃先がやや下向き（浅い掘削・整地）
   *         0.52    |   30  | 刃先が30deg下向き（通常の掘削の食い込み）
   *         0.79    |   45  | 刃先が45deg下向き（深い掘削）
   *         1.57    |   90  | 刃先が真下
   *         2.36    |  135  | 刃先が車体側へ回り込む（かき込み終盤）
   *         3.14    |  180  | 底面が水平、刃先は車体側（バケットを最大に近く曲げた状態）
   *
   * 注意: 実際に到達できる角度はバケット関節の可動域(-0.622〜2.372 rad)とブーム・アームの姿勢で制限される。
   *       範囲外の場合、goal_orientation_tolerance(0.5rad)内なら可動域の限界姿勢でプランが通り、超えるとIKが失敗する。
   *       例: 刃先位置(x=-6.28, z=-1.23)では theta_w が約0.9未満だとバケット関節が下限に達する。
   *
   * output: xyz position and quaternion */
  void convertToXYZQuaternion(const double x, const double y, const double z, const double theta_w, Pose& result_pose);

private:
  // bucket_end_linkのx軸とバケット底面(かかと〜刃先)のなす角 [rad] (zx200-bucket.dae の凸包から算出: 36.7deg)
  static constexpr double kBucketFrameOffset = 0.641;

  // Helper function to convert euler angles to quaternion
  void eulerToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw);
};

#endif  // EXCAVATOR_POSE_CONVERTER_HPP