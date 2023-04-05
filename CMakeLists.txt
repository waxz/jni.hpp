cmake_minimum_required(VERSION 3.16)
project(jni.hpp)

include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


# 设置生成的安装包名字
set(CPACK_PACKAGE_NAME ${PROJECT_NAME})
# 设置支持指定安装目录的控制为 ON
set(CPACK_SET_DESTDIR ON)
# 设置安装到的目录路径
set(CPACK_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
# 这是生成的安装的版本号信息
set(CPACK_PACKAGE_VERSION "1.0.0")
# 设置 group 名字
set(CPACK_RPM_PACKAGE_GROUP "vesoft")
# 设置 vendor 名字
set(CPACK_PACKAGE_VENDOR "vesoft")
# 设置 license 信息
set(CPACK_RPM_PACKAGE_LICENSE "Apache 2.0 + Common Clause 1.0")
set(CPACK_GENERATOR "DEB")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Yan QiDong")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "This is a demo package from Yan QiDong.")
set(CPACK_DEBIAN_PACKAGE_VERSION ${PACKAGE_VERSION})
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
#set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libc6 (<< 3)")


include(cmake/set_ros.cmake)

set(build_ros_lib OFF)
if (build_ros_lib)
    #roscpp
    set(roscpp_ROOT ${ROS_INSTALL_PATH}/share)
    find_package(roscpp REQUIRED )
    #rosconsole
    set(rosconsole_ROOT ${ROS_INSTALL_PATH}/share)
    find_package(rosconsole REQUIRED )
    add_library(ros_lib SHARED examples/ros_lib.cpp)
    message(roscpp_INCLUDE_DIRS: ${roscpp_INCLUDE_DIRS} )
    message(roscpp_LIBRARIES: ${roscpp_LIBRARIES} )
    message(rosconsole_LIBRARIES: ${rosconsole_LIBRARIES} )

    #set(roscpp_libs roscpp rosconsole roscpp_serialization xmlrpcpp rostime cpp_common)
    # rosconsole_backend_interface rosconsole_log4cxx
    add_ros(ros_lib rosconsole roscpp  roscpp_serialization xmlrpcpp rostime cpp_common)
    target_include_directories(ros_lib PUBLIC ${roscpp_INCLUDE_DIRS} )

    add_library(ros_lib_java SHARED examples/ros_lib_java.cpp)

    add_jni(ros_lib_java openjdk)
    target_link_libraries(ros_lib_java PUBLIC ros_lib)


    #set_target_runpath(ros_lib  ${ROS_INSTALL_PATH}/lib)

    add_executable(ros_lib_test  ./examples/ros_lib_test.cpp)
    target_link_libraries(ros_lib_test PUBLIC ros_lib)

endif ()


function(add_jni target variant)
    message(include_dir = ${CMAKE_SOURCE_DIR}/test/${variant})
    target_include_directories(${target} PUBLIC ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/test/${variant} )
endfunction()


set(use_installed_boost OFF)
if(use_installed_boost)
    find_package(Boost
            1.71.0      # Minimum or EXACT version e.g. 1.67.0
            REQUIRED            # Fail with error if Boost is not found
            COMPONENTS thread chrono filesystem
            )

    message(Boost_LIBRARIES = ${Boost_LIBRARIES})

endif()

add_library(path_finding_java SHARED examples/path_finding_java.cpp)
add_jni(path_finding_java openjdk)
target_include_directories(path_finding_java PUBLIC
        ${CMAKE_SOURCE_DIR}/third_party/boost_1_81_0
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann)

add_executable(path_finding_test examples/path_finding_test.cpp)
target_include_directories(path_finding_test PUBLIC
        ${CMAKE_SOURCE_DIR}/third_party/boost_1_81_0
        ${CMAKE_SOURCE_DIR}/third_party/nlohmann)

add_jni(path_finding_test openjdk)

#set_target_runpath(ros_lib_test ${ROS_INSTALL_PATH}/lib)

set(INSTALL_PATH /home/waxz/IdeaProjects/java_test/java_test/native_libs)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib/
        DESTINATION ${INSTALL_PATH}
        PATTERN "launch/data" EXCLUDE
        )
