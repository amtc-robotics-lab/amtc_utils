/** 
 * @file Segment.h 
 * 
 */ 
 
#ifndef AMTC_UTILS_SEGMENT_H
#define AMTC_UTILS_SEGMENT_H

#include <amtc_utils/Vector2d.h> 
 
namespace amtc 
{ 
 
class Segment 
{ 
public: 
 
  Segment(){} 
 
  Segment(const Vector2d& p1, const Vector2d& p2) : 
    p1(p1), 
    p2(p2) 
  { 
    diff = p2 - p1; 
    offset = p1.dot(diff); 
    squaredNorm = diff.dot(diff); 
  } 
 
  double minDistToPoint(const Vector2d& p) 
  { 
    double lambda = ( p.dot(diff) - offset ) / squaredNorm; 
    double lambda2 = std::min(1.0, std::max(0.0, lambda)); 
     
    return (p1 + diff*lambda2 - p).norm(); 
  } 

  bool isInside(const Vector2d& p)
  {
    Vector2d u = (p - p1).normalized();
    Vector2d v = (p - p1).normalized();
    double lambda = ( p.dot(diff) - offset ) / squaredNorm;

    return u.x() == v.x() && u.y() == v.y() && lambda >= 0.0 && lambda <= 1.0;
  }


  bool intersects(const Segment& other)
  {
    if(p1.x() == other.p1.x() && p1.y() == other.p1.y())
      return true;

    const Vector2d u0 = p1;
    const Vector2d v0 = p2 - p1;

    const Vector2d u1 = other.p1;
    const Vector2d v1 = other.p2 - other.p1;

    double x00 = u0.x();
    double y00 = u0.y();
    double x10 = u1.x();
    double y10 = u1.y();

    double x01 = v0.x();
    double y01 = v0.y();
    double x11 = v1.x();
    double y11 = v1.y();

    double det = x11*y01 - x01*y11;
    double s = (1/det) * ((x00 - x10) * y01 - (y00 - y10) * x01);
    double t = (1/det) * ( -(-(x00 - x10) * y11 + (y00 - y10) * x11));

    return s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0;
  }
 
private: 
 
  Vector2d p1; 
  Vector2d p2; 
  Vector2d diff; 
  double offset; 
  double squaredNorm; 
}; 
 
} 

#endif /* AMTC_UTILS_SEGMENT_H*/

