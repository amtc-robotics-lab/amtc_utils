#ifndef AMTC_UTILS_POSE2D_H
#define AMTC_UTILS_POSE2D_H

#include <amtc_utils/Vector2d.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace amtc
{
class Pose2d : public Vector2d
{
public:

  Pose2d() :
    Vector2d(),
    orientation(Vector2d(1.0, 0.0))
  {}

  Pose2d(const Pose2d& o) :
    Vector2d(o),
    orientation(o.orientation)
  {}

  Pose2d(const Vector2d& position, const Vector2d& orientation_) :
    Vector2d(position),
    orientation(orientation_)
  {}

  Pose2d(double x_, double y_, double cos_ori_, double sin_ori_) :
    Vector2d(x_,y_),
    orientation(cos_ori_, sin_ori_)
  {}

  Pose2d(double x_, double y_, double ori) :
    Vector2d(x_,y_),
    orientation(cos(ori), sin(ori))
  {}

  Pose2d(const Vector2d& position, double ori) :
    Vector2d(position), orientation(cos(ori), sin(ori))
  {}

  Pose2d(geometry_msgs::msg::Pose pose_msg) :
    Vector2d(),
    orientation(1.0, 0.0)
  {
    fromPoseMsg(pose_msg);
  }

  inline Vector2d& position()
  {
    return *this;
  }

  inline const Vector2d& position() const
  {
    return *this;
  }

  inline bool isZero() const 
  {
    return orientation.isZero() && Vector2d::isZero();
  }

  inline double getOrientationAngle() const
  {
    return orientation.getAngle();
  }

  inline Pose2d operator=(const Pose2d& pose_2d)
  {
    x() = pose_2d.x();
    y() = pose_2d.y();
    orientation = pose_2d.orientation;
    return *this;
  }

  inline Pose2d operator+(const Vector2d& sum) const
  {
    return Pose2d(position() + sum, orientation);
  }

  inline Pose2d operator-(const Vector2d &sum) const
  {
    return Pose2d(position() - sum, orientation);
  }

  inline Pose2d rotated(const Vector2d& rotation_vec) const
  {
    return Pose2d(Vector2d::rotated(rotation_vec), orientation.rotated(rotation_vec));
  }

  inline Pose2d rotated(double angle) const
  {
    return rotated(Vector2d::polarVector(1.0, angle));
  }

  inline Pose2d &rotate(const Vector2d& rotation_vec)
  {
    Vector2d::rotate(rotation_vec);
    orientation.rotate(rotation_vec);

    return *this;
  }

  inline std::string to_string() const
  {
    return Vector2d::to_string() + std::string(",") + orientation.to_string();
  }

  inline static Pose2d Zero()
  {
    return Pose2d(Vector2d::Zero() , Vector2d(1.0, 0.0));
  }

  // inline static Pose2d fromTf(const tf::Pose& pose)
  // {
  //   return amtc::Pose2d(amtc::Vector2d(pose.getOrigin().x(), pose.getOrigin().y()), amtc::Vector2d::polarVector(1.0, tf::getYaw(pose.getRotation())));
  // }

  // inline tf::Pose toTf()
  // {
  //   tf::Pose pose;
  //   pose.setOrigin(tf::Vector3(position().x(), position().y(), 0.f));
  //   pose.setRotation(tf::createQuaternionFromYaw(getOrientationAngle()));
  //   return pose;
  // }

  inline void fromPoseMsg(geometry_msgs::msg::Pose pose_msg)
  {
    position().x() = pose_msg.position.x;
    position().y() = pose_msg.position.y;
    tf2::Quaternion q(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
    float angle = tf2::getYaw(q);
    orientation.x() = cos(angle);
    orientation.y() = sin(angle);
    return;
  }

  inline geometry_msgs::msg::Pose toPoseMsg()
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = position().x();
    pose.position.y = position().y();
    tf2::Quaternion q;
    q.setEuler(0, 0, getOrientationAngle());
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  inline geometry_msgs::msg::PoseStamped toPoseStampedMsg()
  {
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.pose = toPoseMsg();
    return pose_stamp;
  }

  inline geometry_msgs::msg::PoseStamped toPoseStampedMsg(std::string frame, builtin_interfaces::msg::Time stamp)
  {
    geometry_msgs::msg::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = frame;
    pose_stamp.header.stamp = stamp;
    pose_stamp.pose = toPoseMsg();
    return pose_stamp;
  }

public:

  Vector2d orientation;
};

template <class PositionType>
class Relative
{
public:

  Relative() :
    origin(Pose2d::Zero()),
    relative(PositionType::Zero()),
    absolute(PositionType::Zero())
  {}

  Relative(const Relative& o) :
    origin(o.origin),
    relative(o.relative),
    absolute(o.absolute)
  {}

  Relative(const PositionType& pos) :
    origin(Pose2d::Zero()),
    relative(pos),
    absolute(pos)
  {}

  Relative(const Pose2d& orig, const PositionType& abs_pos) :
    origin(orig),
    relative(),
    absolute(abs_pos)
  {
    calcThisRelativePosition();
  }

  inline void setAbsolutePosition(const PositionType& pos)
  {
    absolute = pos;
    calcThisRelativePosition();
  }
  
  inline bool isZero() const
  {
    return origin.isZero() && absolute.isZero() && relative.isZero();
  }

  inline void setOrigin(const Pose2d& o)
  {
    origin = o;
    setAbsolutePosition(absolute);
  }

  inline void setRelativePosition(const PositionType& pos)
  {
    relative = pos;
    calcThisAbsolutePosition();
  }

  inline PositionType getRelativeTo(const Pose2d& o) const
  {
    return calcRelativePosition(o, absolute);
  }

  inline const PositionType& getAbsolutePosition() const
  {
    return absolute;
  }

  inline const PositionType& getRelativePosition() const
  {
    return relative;
  }

  inline const Pose2d& getOrigin() const
  {
    return origin;
  }

private:

  inline static PositionType calcAbsolutePosition(const Pose2d& orig, const PositionType& rel_pos)
  {
    return rel_pos.rotated(orig.orientation)+orig.position();
  }

  inline void calcThisAbsolutePosition()
  {
    absolute = calcAbsolutePosition(origin, relative);
  }

  inline static PositionType calcRelativePosition(const Pose2d& orig, const PositionType& abs_pos)
  {
    return (abs_pos - orig.position()).rotated(orig.orientation.h_mirror());
  }

  inline void calcThisRelativePosition()
  {
    relative = calcRelativePosition(origin, absolute);
  }

private:

  Pose2d origin;
  PositionType relative;
  PositionType absolute;
};

class Pose2dWithDirId : public Pose2d
{
public:
  Pose2dWithDirId() :
    Pose2d(),
    dirId(-1)
  {}

  Pose2dWithDirId(const Pose2d& o) :
    Pose2d(o),
    dirId(-1)
  {}

  Pose2dWithDirId(const Pose2d& o, int dir) :
    Pose2d(o),
    dirId(dir)
  {}

  Pose2dWithDirId(const Pose2dWithDirId& o) :
    Pose2d(o),
    dirId(o.dirId)
  {}

public:

  int dirId;
};

}

#endif /* AMTC_UTILS_POSE2D_H */
