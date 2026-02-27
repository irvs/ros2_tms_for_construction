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

  /* input: position of bucket_edge, バケットが水平面に対して成す角[rad] */
  /* output: xyz position and quaternion */
  void convertToXYZQuaternion(const double x, const double y, const double z, const double theta_w, Pose& result_pose);

private:
  // Helper function to convert euler angles to quaternion
  void eulerToQuaternion(double roll, double pitch, double yaw, double& qx, double& qy, double& qz, double& qw);
};

#endif  // EXCAVATOR_POSE_CONVERTER_HPP