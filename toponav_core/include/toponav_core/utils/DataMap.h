#pragma once

#include <boost/any.hpp>
#include <map>

namespace toponav_core {

/**
 * @brief User Data Map. maps string => boost::any.
 */
class DataMap: public std::map<std::string, boost::any> {
  static const boost::any dummy;
public:
  using map::map;
  
  /**
   * returns either the value under given key string an empty boost::any if key not present.
   * @param key key string
   * @return this->at(key) or reference to an empty dummy value if key not in map.
   */
  const boost::any& getOrEmpty(key_type const& key) const{
    auto iter = this->find(key);
    if(iter == this->end())
      return dummy;
    else
      return iter->second;
  }
};
  
} // namespace toponav_core