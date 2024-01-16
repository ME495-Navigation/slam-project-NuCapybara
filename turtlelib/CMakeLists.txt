cmake_minimum_required(VERSION 3.22)


project(project_name)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)


add_library(libname src/libfile1.cpp src/libfile2.cpp)


target_include_directories(libname
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)


target_compile_features(libname PUBLIC cxx_std_17)

add_executable(Name src/Name_main.cpp)

target_link_libraries(Name Eigen3::Eigen libname)


install(DIRECTORY include/mylibrary DESTINATION include)

install(TARGETS Name libname EXPORT project_name-targets)

install(EXPORT project_name-targets
        FILE project_name-config.cmake
        NAMESPACE project_name::
        DESTINATION lib/cmake/${PROJECT_NAME})

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