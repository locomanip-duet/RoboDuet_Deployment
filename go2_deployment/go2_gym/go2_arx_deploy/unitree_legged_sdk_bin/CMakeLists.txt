cmake_minimum_required(VERSION 3.5)
project(lcm_position)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

set(ARX_cpp_source
    arx/src/utility.cpp
    arx/src/Hardware/motor.cpp
    arx/src/Hardware/math_ops.cpp
    arx/src/App/arm_control.cpp
)

set(ARX_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/arx)
add_definitions(-DARX_DIRECTORY="${ARX_DIRECTORY}")


include_directories(
    include
    ../lcm_types
    /usr/include/eigen3
    ${CMAKE_CURRENT_SOURCE_DIR}/arx/include
    ${CMAKE_CURRENT_SOURCE_DIR}/arx/src
)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
  set(ARM_ARCH x86_64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
  set(ARM_ARCH arm64)
endif()

link_directories(
  arx/libcan/${ARCH}
)
  
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})


# add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)
add_executable(lcm_position_vr_go2 lcm_position_vr_go2.cpp ${ARX_cpp_source})
target_link_libraries(lcm_position_vr_go2 lcm)



if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    target_link_libraries(lcm_position_vr_go2
        ${EXTRA_LIBS}
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libarmcan.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libarmscan.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libarmarx_s.so
    )

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    target_link_libraries(lcm_position_vr_go2
        ${EXTRA_LIBS}
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libcana.so
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libcan.a
        ${CMAKE_CURRENT_SOURCE_DIR}/arx/libcan/${ARCH}/libarx_s.so
    )

endif()



set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "ARX_CPP")
set(CPACK_PACKAGE_VERSION "0.0.1")
set(CPACK_PACKAGE_CONTACT "js011107@163.com")
set(CMAKE_BUILD_TYPE Debug)
include(CPack)