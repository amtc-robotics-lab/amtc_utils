#pragma once

namespace amtc {
class Resource
{
public:
  virtual ~Resource() {}
  
  virtual bool isAvailable() = 0;
  
};

}