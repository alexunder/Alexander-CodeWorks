cmake_minimum_required(VERSION 3.5)
project(kayak_test)

set(CMAKE_CXX_STANDARD 11)


set(INSTALL_GTEST OFF)
add_subdirectory(googletest)

include_directories("./eigen-3.3.7")
include_directories("./Utils")

set(CMAKE_CXX_FLAGS "-g -rdynamic")

set(SRC
    main.cpp
    KayakTest.cpp
    Algorithm.cpp
    RoboticsDynamics.cpp
    Algorithm.cpp
    KukaAlgorithms.cpp
	hls_helper.cpp
    Quaternion.cpp
    Utils/UtilFunc.cpp
    Utils/GeometryUtils.cpp
    Utils/Frame.cpp
    Utils/Rotation.cpp
    )

add_executable(kayak_test ${SRC})
target_link_libraries(kayak_test gtest)
