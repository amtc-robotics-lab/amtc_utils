/*
 * Utils.cpp
 *
 *  Created on: May 30, 2014
 *      Author: isao
 */

#include <amtc_utils/Utils.h>
#include <iostream>
namespace amtc{


std::vector<std::string> &split_string(const std::string &s, char delim, std::vector<std::string> &elems)
{
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim))
  {
    elems.push_back(item);
  }
  return elems;
}

std::vector<std::string> split_string(const std::string &s, char delim)
{
  std::vector<std::string> elems;
  split_string(s, delim, elems);
  return elems;
}

  unsigned char char_to_number(unsigned char c)
  {
    switch (c)
    {
      case '0':
        return 0;
      case '1':
        return 1;
      case '2':
        return 2;
      case '3':
        return 3;
      case '4':
        return 4;
      case '5':
        return 5;
      case '6':
        return 6;
      case '7':
        return 7;
      case '8':
        return 8;
      case '9':
        return 9;
      case 'A':
      case 'a':
        return 10;
      case 'B':
      case 'b':
        return 11;
      case 'C':
      case 'c':
        return 12;
      case 'D':
      case 'd':
        return 13;
      case 'E':
      case 'e':
        return 14;
      case 'F':
      case 'f':
        return 15;
      default:
        return 0;
    }
  }

  void set_DataConverterType(const std::string &s, DataConverterType& data)
  {
    unsigned int i;
    unsigned int s_len = s.size();
    data.uint64_data = 0;

    for (i = 0; i < s_len && i < 16; ++i)
    {
      data.uint64_data = (data.uint64_data << 4) + char_to_number( s[i] );
    }
  }

uint8_t string_to_uint8_t(const std::string &s, unsigned int base)
{

  return (uint8_t) std::stoul(s,nullptr, 16);
}

uint16_t string_to_uint16_t(const std::string &s, unsigned int base)
{
  return (uint16_t) std::stoul(s,nullptr, 16);;
}

uint32_t string_to_uint32_t(const std::string &s, unsigned int base)
{
  return (uint32_t) std::stoul(s,nullptr, 16);
}

int8_t string_to_int8_t(const std::string &s, unsigned int base)
{
  return (int8_t) std::stol(s,nullptr, 16);
}

int16_t string_to_int16_t(const std::string &s, unsigned int base)
{
  return (int16_t) std::stol(s,nullptr, 16);
}

int32_t string_to_int32_t(const std::string &s, unsigned int base)
{
  return (int32_t) std::stol(s,nullptr, 16);
}

float   string_to_float(const std::string &s)
{
  static DataConverterType data;
  set_DataConverterType(s, data );
  return data.float32_data;
}





} /* namespace amtc */

