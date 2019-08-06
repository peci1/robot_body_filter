# we're using std::optional or std::experimental
# CMake 3.7 in Debian Stretch doesn't know C++17 yet
set(CMAKE_CXX_STANDARD 14)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.8")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckCXXSourceCompiles)

# support std::optional
set(OLD_CMAKE_REQUIRED_FLAGS ${CMAKE_REQUIRED_FLAGS})
set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS} -std=c++17)
check_cxx_source_compiles("
  #include <optional>
  #include <string>

  int main()
  {
    std::optional< std::string > a;
    std::string b = a.value_or( \"empty\" );
  }
" ROBOT_BODY_FILTER_HAVE_CXX_OPTIONAL
  )

# support for std::experimental::optional
set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS} -std=c++14)
check_cxx_source_compiles("
  #include <experimental/optional>
  #include <string>

  int main()
  {
    std::experimental::optional< std::string > a;
    std::string b = a.value_or( \"empty\" );
  }
" ROBOT_BODY_FILTER_HAVE_CXX_EXPERIMENTAL_OPTIONAL
  )

set(CMAKE_REQUIRED_FLAGS ${OLD_CMAKE_REQUIRED_FLAGS})

if(NOT "${ROBOT_BODY_FILTER_HAVE_CXX_OPTIONAL}" AND NOT "${ROBOT_BODY_FILTER_HAVE_CXX_EXPERIMENTAL_OPTIONAL}")
  message(FATAL "Your compiler is too old. It has to support either std::experimental::optional or std::optional.")
endif()