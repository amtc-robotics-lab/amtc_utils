#ifndef AMTC_UTILS_TRANSFORM2D_H
#define AMTC_UTILS_TRANSFORM2D_H

#include <amtc_utils/Vector2d.h>
#include <amtc_utils/Pose2d.h>

namespace amtc
{

class Transform2d
{
public:
  Transform2d() :
    origin(0.0, 0.0),
    rotation(1.0, 0.0)
  {}

  Transform2d(double tx_, double ty_, double cos_ori_, double sin_ori_) :
    origin(tx_, ty_),
    rotation(cos_ori_, sin_ori_)
  {
    rotation.normalize();
  }

  Transform2d(const Vector2d& origin_, const Vector2d& rotation_) :
    origin(origin_),
    rotation(rotation_)
  {
    rotation.normalize();
  }

  Transform2d(const Pose2d& pose) :
    origin(pose.position()),
    rotation(pose.orientation)
  {}

  inline Transform2d o(const Transform2d& tr) const
  {
    Vector2d otherOrigin = tr.origin + origin.rotated(tr.rotation);
    Vector2d otherRotation = rotation.rotated(tr.rotation);
    return Transform2d(otherOrigin, otherRotation);
  }

  inline Transform2d inverse() const
  {
    return Transform2d(*this).invert();
  }

  inline Transform2d& invert()
  {
    rotation = rotation.h_mirror();
    origin = -origin.rotated(rotation);
    return *this;
  }

  inline Vector2d operator()(const Vector2d& point) const
  {
    return origin + point.rotated(rotation);
  }

  inline Pose2d operator()(const Pose2d& pose) const
  {
    return Pose2d(this->operator()(pose.position()), pose.getOrientationAngle() + rotation.getAngle());
  }

  inline Transform2d &operator= (const Pose2d& pose)
  {
    origin = pose.position();
    rotation = pose.orientation;

    return *this;
  }

  inline operator Pose2d() const
  {
    return Pose2d(origin, rotation);
  }

  // inline static Transform2d fromTf(const tf::Pose& pose)
  // {
  //   return amtc::Transform2d(amtc::Vector2d(pose.getOrigin().x(), pose.getOrigin().y()), amtc::Vector2d::polarVector(1.0, tf::getYaw(pose.getRotation())));
  // }

  // inline static Transform2d fromTf(const geometry_msgs::Pose& msg)
  // {
  //   tf::Pose pose;
  //   tf::poseMsgToTF(msg, pose);

  //   return fromTf(pose);
  // }

  // inline tf::Pose toTf()
  // {
  //   tf::Pose pose;
  //   pose.setOrigin(tf::Vector3(origin.x(), origin.y(), 0.f));
  //   pose.setRotation(tf::createQuaternionFromYaw(rotation.getAngle()));

  //   return pose;
  // }

public:

  Vector2d origin;
  Vector2d rotation; //(cos_ori,sin_ori)
};

}
#endif /* AMTC_UTILS_TRANSFORM2D_H_ */
