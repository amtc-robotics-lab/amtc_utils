/**
 * @file Vector2d.h
 *
 * @author Paul Vallejos
 * @date 2016/04/04 (creation)
 *
 */

#ifndef AMTC_UTILS_VECTOR2D_H
#define AMTC_UTILS_VECTOR2D_H

#include <boost/numeric/ublas/vector.hpp>

#include <cmath>
#include <string>

namespace amtc
{
typedef boost::numeric::ublas::vector< double,boost::numeric::ublas::bounded_array<double, 2> > ublas_bounded_vector2;

class Vector2d : ublas_bounded_vector2
{
public:

  Vector2d(double x_,double y_) :
    ublas_bounded_vector2(2)
  {
    x() = x_;
    y() = y_;
  }

  Vector2d() :
   Vector2d(0.0, 0.0)
  {}

  Vector2d(const Vector2d& o) : ublas_bounded_vector2(o){}

  inline double& x(){return this->operator()(0);}
  inline double& y(){return this->operator()(1);}
  inline double x() const{return this->operator()(0);}
  inline double y() const{return this->operator()(1);}

  inline Vector2d operator=(const Vector2d& vec)
  {
    x() = vec.x();
    y() = vec.y();
    return *this;
  }

  inline Vector2d operator+(const Vector2d& sum) const
  {
    return Vector2d(x() + sum.x(), y() + sum.y());
  }

  inline Vector2d& operator+=(const Vector2d& sum)
  {
    x() += sum.x();
    y() += sum.y();

    return *this;
  }

  inline Vector2d operator-(const Vector2d& sum) const
  {
    return Vector2d(x() - sum.x(), y() - sum.y());
  }

  inline Vector2d &operator-=(const Vector2d &sum)
  {
    x() -= sum.x();
    y() -= sum.y();

    return *this;
  }

  inline Vector2d operator-() const
  {
    return Vector2d(-x(), -y());
  }

  inline Vector2d& operator*=(double factor)
  {
    x() *= factor;
    y() *= factor;

    return *this;
  }

  inline Vector2d operator*(double factor) const
  {
    return Vector2d(x()*factor, y()*factor);
  }

  inline Vector2d &operator/=(double factor)
  {
    x() /= factor;
    y() /= factor;

    return *this;
  }

  inline Vector2d operator/(double factor) const
  {
    return Vector2d( x()/factor, y()/factor );
  }

  inline double norm() const{return std::sqrt(x()*x() + y()*y());}
  inline double norm2() const{return x()*x() + y()*y();}

  inline Vector2d& normalize()
  {
    double n = norm();

    if(n == 0)
    {
      x() = 1.0;
      y() = 0.0;
    }
    else
    {
      operator/=(n);
    }

    return *this;
  }

  inline Vector2d normalized() const
  {
    Vector2d result(*this);
    result.normalize();

    return result;
  }

  inline Vector2d rotated(const Vector2d& rotation_vec) const
  {
    return Vector2d((x()*rotation_vec.x() - y()*rotation_vec.y())/*/norm*/,
                    (x()*rotation_vec.y() + y()*rotation_vec.x())/*/norm*/);
  }

  inline Vector2d rotated(double angle) const
  {
    return rotated(polarVector(1.0, angle));
  }

  inline Vector2d& rotate(const Vector2d& rotation_vec)
  {
    *this = this->rotated(rotation_vec);

    return *this;
  }

  inline Vector2d& rotate(double angle)
  {
    *this = this->rotated(angle);

    return *this;
  }

  inline Vector2d h_mirror() const
  {
    return Vector2d(x(), -y());
  }

  inline double getAngle() const{ return std::atan2(y(),x());}

  inline double getTanAngle() const{return y()/x();}

  inline double dot(const Vector2d& o) const
  {
    return x()*o.x() + y()*o.y();
  }

  inline std::string to_string() const
  {
    return  std::string("[") + std::to_string(x()) + std::string(",") + std::to_string(y()) + std::string("]");
  }

  inline static Vector2d polarVector(double norm, double angle)
  {
    return Vector2d(norm*std::cos(angle), norm*std::sin(angle));
  }

  inline static Vector2d Zero()
  {
    return Vector2d(0.0, 0.0);
  }
};

}
#endif /* AMTC_UTILS_VECTOR2D_H_ */
