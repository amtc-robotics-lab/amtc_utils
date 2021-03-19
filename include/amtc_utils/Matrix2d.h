//
//  Vector2d.h
//
//  Created by Paul Vallejos on 04-04-16.
//
//

#ifndef AMTC_UTILS_MATRIX2D_H
#define AMTC_UTILS_MATRIX2D_H

#include <boost/numeric/ublas/matrix.hpp>

namespace amtc {
typedef boost::numeric::ublas::matrix<double, boost::numeric::ublas::row_major, boost::numeric::ublas::bounded_array<double, 4>> ublas_bounded_matrix4;
class Matrix2d : public ublas_bounded_matrix4 {
public:
    Matrix2d():ublas_bounded_matrix4(2,2){};
    Matrix2d(const ublas_bounded_matrix4 &o):ublas_bounded_matrix4(o){};
    Matrix2d(const Matrix2d &o):ublas_bounded_matrix4(o){};
    
    /* i: fila, j: columna */
    inline double &operator()(unsigned int i, unsigned int j){return ublas_bounded_matrix4::operator()(i,j);};
    inline double operator()(unsigned int i, unsigned int j) const {return ublas_bounded_matrix4::operator()(i,j);};
    
    inline Matrix2d operator+(const Matrix2d &sum) const{
        Matrix2d result(*this);
        return (result+=(sum));
    };
    inline Matrix2d &operator+=(const Matrix2d &sum) {
        ublas_bounded_matrix4::operator+=(sum);
        return *this;
    };
    inline Matrix2d operator-(const Matrix2d &sum) const{
        Matrix2d result(*this);
        return (result-=(sum));
    };
    inline Matrix2d &operator-=(const Matrix2d &sum) {
        ublas_bounded_matrix4::operator-=(sum);
        return *this;
    };
    inline Matrix2d operator-() const{
        Matrix2d result(boost::numeric::ublas::zero_matrix<double>(2,2));
        return (result-=*this);
    };
    inline Matrix2d &operator*=(double factor) {
        ublas_bounded_matrix4::operator*=(factor);
        return *this;
    };
    inline Matrix2d operator*(double factor) const{
        Matrix2d result(*this);
        return (result*=(factor));
    };
    inline Matrix2d &operator/=(double factor) {
        ublas_bounded_matrix4::operator/=(factor);
        return *this;
    };
    inline Matrix2d operator/(double factor) const{
        Matrix2d result(*this);
        return (result/=(factor));

    };
    inline double det()const{
        return operator()(0,0)*operator()(1,1)-operator()(0,1)*operator()(1,0);
    };
    inline std::string to_string() const{
        return  std::string("[") + std::to_string(operator()(0,0)) + std::string(",") + std::to_string(operator()(0,1)) + std::string(";") + std::to_string(operator()(1,0)) + std::string(",") + std::to_string(operator()(1,1)) + std::string("]");
    };
    inline static Matrix2d zero(){
        return Matrix2d( boost::numeric::ublas::zero_matrix<double, boost::numeric::ublas::bounded_array<double,4>>(2, 2) );
    };
};
};
#endif /* AMTC_UTILS_MATRIX2D_H */
