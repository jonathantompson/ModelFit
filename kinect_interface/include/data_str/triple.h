//
//  triple.h
//
//  Created by Jonathan Tompson on 1/5/13.
//

#pragma once

namespace jtil {
namespace data_str {

  template <typename TFirst, typename TSecond, typename TThird>
  class Triple {
  public:
    Triple() { }
    Triple(const TFirst& _first, const TSecond& _second,
      const TSecond& _third) : first(_first), second(_second), 
      third(_third) { }

    bool operator==(const Triple<TFirst, TSecond, TThird>& a) const;
    void operator=(const Triple<TFirst, TSecond, TThird>& a);

    TFirst first;
    TSecond second;
    TThird third;
  };

  template <typename TFirst, typename TSecond, typename TThird>
  bool Triple<TFirst, TSecond, TThird>::operator==(const Triple<TFirst, 
    TSecond, TThird>& a) 
    const {
    if (this == &a) {  // if both point to the same memory
      return true; 
    }
    if (first == a.first && second == a.second && third == a.third) { 
      return true; 
    } else {
      return false;
    }
  };

  template <typename TFirst, typename TSecond, typename TThird>
  void Triple<TFirst, TSecond, TThird>::operator=(const Triple<TFirst, 
    TSecond, TThird>& a) {
    if (this == &a) {  // if both point to the same memory
      return; 
    }
    first = a.first;
    second = a.second;
    third = a.third;
  };

};  // namespace data_str
};  // namespace jtil
