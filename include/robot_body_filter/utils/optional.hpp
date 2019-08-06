#ifndef ROBOT_BODY_FILTER_OPTIONAL_HPP
#define ROBOT_BODY_FILTER_OPTIONAL_HPP

#if ROBOT_BODY_FILTER_USE_CXX_OPTIONAL
#include <optional>
using std::optional;
using std::nullopt;
#else
#include <experimental/optional>
using std::experimental::optional;
using std::experimental::nullopt;
#endif

#endif //ROBOT_BODY_FILTER_OPTIONAL_HPP
