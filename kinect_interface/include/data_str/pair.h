//
//  pair.h
//
//  Created by Jonathan Tompson on 4/26/12.
//

#pragma once

namespace jtil {
namespace data_str {

  template <typename TFirst, typename TSecond>
  class Pair {
  public:
    Pair() { }
    Pair(const TFirst& _first, const TSecond& _second) : first(_first), 
      second(_second) { }

    bool operator==(const Pair<TFirst, TSecond>& a) const;
    void operator=(const Pair<TFirst, TSecond>& a);

    TFirst first;
    TSecond second;
  };

  template <typename TFirst, typename TSecond>
  bool Pair<TFirst, TSecond>::operator==(const Pair<TFirst, TSecond>& a) 
    const {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (first == a.first && second == a.second) { 
      return true; 
    } else {
      return false;
    }
  };

  template <typename TFirst, typename TSecond>
  void Pair<TFirst, TSecond>::operator=(const Pair<TFirst, TSecond>& a) {
    if (this == &a) {  // if both point to the same memory
      return; 
    }
    first = a.first;
    second = a.second;
  };

};  // namespace data_str
};  // namespace jtil
