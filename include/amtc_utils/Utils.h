/*
 * Utils.h
 *
 *  Created on: May 30, 2014
 *      Author: isao
 */

#ifndef AMTC_UTILS_UTILS_H_
#define AMTC_UTILS_UTILS_H_

#include <vector>
#include <string>
#include <sstream>
#include <inttypes.h>
#include <ctime>
#include <rclcpp/rclcpp.hpp>

namespace amtc {


  std::string  to_string(const rclcpp::Time &time, bool include_ns = false);
/*
 * std::vector<std::string> &split_string(const std::string &s, char delim, std::vector<std::string> &elems)
 * Description:
 *     Convierte un string en un vector con strings separados por un delimitar.
 * Input:
 *     const std::string &s:
 *         string de entrada, el que se va a dividir
 *     char delim:
 *         caracter delimitador.
 *     std::vector<std::string> &elems:
 *         referencia al vector con los elementos divididos
 * Output:
 *     std::vector<std::string>&:
 *         referencia al vector con los elementos divididos
 */
std::vector<std::string> &split_string(const std::string &s, char delim, std::vector<std::string> &elems);
/*
 * std::vector<std::string> split_string(const std::string &s, char delim)
 * Description:
 *     Convierte un string en un vector con strings separados por un delimitar.
 * Input:
 *     const std::string &s:
 *         string de entrada, el que se va a dividir
 *     char delim:
 *         caracter delimitador.
 * Output:
 *     std::vector<std::string>&:
 *         referencia al vector con los elementos divididos
 */
std::vector<std::string> split_string(const std::string &s, char delim);

/*
 * template <typename M> M linear_map(M value, M min_limit, M max_limit, M new_min_limit, M new_max_limit)
 * Description:
 *     mapea linealmente un valor en los limites de entrada a los nuevos limites
 * Input:
 *     M value:
 *         valor a mapear
 *     M min_limit:
 *         limite inferior de los valores de entrada
 *     M max_limit:
 *         limite superior de los valores de entrada
 *     M new_min_limit:
 *         limite inferior de los valores de salida
 *     M new_max_limit:
 *         limite superior de los valores de salida

 * Output:
 *     M:
 *         valor mapeado de salida
 */
template <typename M> M linear_map(M value, M min_limit, M max_limit, M new_min_limit, M new_max_limit)
{
  return ( value - min_limit ) * ( new_max_limit - new_min_limit ) / ( max_limit - min_limit ) + new_min_limit;
}

/*
 * template <typename M> M linear_map_saturated(M value, M min_limit, M max_limit, M new_min_limit, M new_max_limit)
 * Description:
 *     mapea linealmente un valor en los limites de entrada a los nuevos limites
 * Input:
 *     M value:
 *         valor a mapear
 *     M min_limit:
 *         limite inferior de los valores de entrada
 *     M max_limit:
 *         limite superior de los valores de entrada
 *     M new_min_limit:
 *         limite inferior de los valores de salida
 *     M new_max_limit:
 *         limite superior de los valores de salida

 * Output:
 *     M:
 *         valor mapeado de salida
 */
template <typename M> M linear_map_saturated(M value, M min_limit, M max_limit, M new_min_limit, M new_max_limit)
{
  if (value <= min_limit)
  {
    return new_min_limit;
  }
  if (value >= max_limit)
  {
    return new_max_limit;
  }
  return linear_map(value, min_limit, max_limit, new_min_limit, new_max_limit);
}

/*
 * template <class M> std::string eigen_matrix_to_string(M& matrix,char delim='|')
 * Description:
 *     Convierte una matriz eigen en un string que contiene su dimension y los elementos, separados por delim.
 * Input:
 *     M& matrix:
 *         referencia a matriz a transformar en string.
 *     char delim:
 *         char delimitador.
 *
 * Output:
 *     std::vector<std::string>:
 *         string con la matriz
 */
template <class M>
std::string eigen_matrix_to_string(M& matrix,char delim='|')
{
  std::ostringstream ss;
  ss << matrix.rows() << delim << matrix.cols() << delim;
  for(unsigned int i=0; i< matrix.rows();++i)
  {
    for(unsigned int j=0; j< matrix.cols();++j)
    {
      ss<<matrix(i,j);
      if ( i != (matrix.rows()-1) || j != (matrix.cols()-1) )
      {
        ss<<delim;
      }
    }
  }
  return ss.str();
}


  /*
 * template <class M> bool string_to_eigen_matrix(const std::string& str,M& matrix,char delim='|')

 * Description:
 *     Convierte lee el string str y lo guarda en la matriz matrix,
 *     los dos primeros elementos son las dimensiones, los siguientes son los elementos separados por el char delim.
 * Input:
 *     const std::string &s:
 *         string de entrada
 *     M& matrix:
 *         referencia a matriz a donde guardar los elementostransformar en string.
 *     char delim:
 *         char delimitador.
 *
 * Output:
 *     bool:
 *         si se convierte exitosamente, en caso que los elementos falten o las dimensiones no coincidan reportara falso.
 */

template <class M>
bool string_to_eigen_matrix(const std::string& str,M& matrix,char delim='|',bool resize=false)
{
  std::vector<std::string> elements = split_string(str,delim);
  if( elements.size()<=2 )  // no suficientes elementos
  {
    return false;
  }
  unsigned int i_size;
  unsigned int j_size;
  {
    std::istringstream istr(elements[0]);
    istr >> i_size;
  }
  {
    std::istringstream istr(elements[1]);
    istr >> j_size;
  }
  if( resize == true )
  {
    matrix.resize(i_size,j_size);
  }

  if( elements.size() != (2+i_size*j_size) )  // no tienen la misma cantidad de elementos
  {
    return false;
  }

  if( i_size != matrix.rows() || j_size != matrix.cols() ) // no coinciden las dimensiones
  {
    return false;
  }

  for(unsigned int i=0; i< matrix.rows();++i)
  {
    for(unsigned int j=0; j< matrix.cols();++j)
    {
      std::istringstream istr(elements[ 2 + i*j_size + j ]);
      istr >> matrix(i,j);
    }
  }
  return true;
}

/*
 * uint16_t string_to_uint16(const std::string &s)
 * Description:
 *     Convierte un string en un uint16_t.
 * Input:
 *     const std::string &s:
 *         string de entrada, el que se va a convertir
 *     unsigned int base:
 *         base numerica para la conversion
 * Output:
 *     uint16_t:
 *         valor numerico del string
 */

  uint8_t   string_to_uint8_t (const std::string &s, unsigned int base = 16 );
  uint16_t  string_to_uint16_t(const std::string &s, unsigned int base = 16 );
  uint32_t  string_to_uint32_t(const std::string &s, unsigned int base = 16 );

  int8_t    string_to_int8_t  (const std::string &s, unsigned int base = 16 );
  int16_t   string_to_int16_t (const std::string &s, unsigned int base = 16 );
  int32_t   string_to_int32_t (const std::string &s, unsigned int base = 16 );

  float     string_to_float   (const std::string &s);

  typedef union
  {
    unsigned char       raw_data[8];
    uint8_t             uint8_data;
    uint16_t            uint16_data;
    uint32_t            uint32_data;
    uint64_t            uint64_data;
    int8_t              int8_data;
    int16_t             int16_data;
    int32_t             int32_data;
    int64_t             int64_data;
    float               float32_data;
    float               float64_data;
  } DataConverterType;

/*
 * bool check_bound(double value, double min_limit, double max_limit)
 * Description:
 *     Evalua si un valor esta dentro de un rango
 * Input:
 *     double value:
 *         valor a evaluar
 *     double min_limit:
 *         limite inferior
 *     double max_limit:
 *         limite superior
 * Output:
 *     bool:
 *         true si esta dentro del rango, falso si no.
 */
inline bool check_bound(double value, double min_limit, double max_limit)
{
  if(value < min_limit || max_limit < value)
  {
    return false;
  }

  return true;
}


} /* namespace amtc */

#endif /* AMTC_UTILS_UTILS_H_ */
