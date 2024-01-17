cmake_minimum_required(VERSION 3.22)
project(turtlelib)
add_library(turtlelib src/geometry2d.cpp)


target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# enable C++ 17
target_compile_features(turtlelib PUBLIC cxx_std_17) 

# warnings are your friend!
target_compile_options(turtlelib PUBLIC -Wall -Wextra -Wpedantic)

install(DIRECTORY include/turtlelib DESTINATION include)

find_package(Catch2 3 REQUIRED)

include(CTest)
add_executable(turtlelib_test tests/test_geometry2d.cpp)
target_link_libraries(turtlelib_test turtlelib Catch2::Catch2WithMain)
add_test(NAME Test_of_Turtlelib COMMAND turtlelib_test)

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

