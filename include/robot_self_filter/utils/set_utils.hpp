#ifndef ROBOT_SELF_FILTER_SET_UTILS_HPP
#define ROBOT_SELF_FILTER_SET_UTILS_HPP

namespace robot_self_filter
{

template<typename T>
bool isSetIntersectionEmpty(const std::set<T>& set1,
                            const std::set<T>& set2) {
  std::set<T> tmpSet;
  std::set_intersection(
      set1.begin(), set1.end(),
      set2.begin(), set2.end(),
      std::inserter(tmpSet, tmpSet.end())
  );
  return tmpSet.empty();
}

}

#endif //ROBOT_SELF_FILTER_SET_UTILS_HPP
