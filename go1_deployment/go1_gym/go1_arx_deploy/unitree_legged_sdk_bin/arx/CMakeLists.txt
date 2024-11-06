#cmake_minimum_required( VERSION 2.8 )
cmake_minimum_required( VERSION 3.16.3 )

project( arx_5cpp )

set(CMAKE_BUILD_TYPE Debug)

set(cpp_source
    src/arx_5v.cpp
    src/utility.cpp
    src/Hardware/motor.cpp
    src/Hardware/math_ops.cpp
    src/App/arm_control.cpp
)

set(ARX_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_definitions(-DARX_DIRECTORY="${ARX_DIRECTORY}")

add_executable(arx_5cpp ${cpp_source})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    target_link_libraries(arx_5cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/libcan/libarmcana.so
        ${CMAKE_CURRENT_SOURCE_DIR}/libcan/libarmcan.a
        ${CMAKE_CURRENT_SOURCE_DIR}/src/App/libarmkey.so
        pthread
    )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    target_link_libraries(arx_5cpp 
        ${CMAKE_CURRENT_SOURCE_DIR}/libcan/libcana.so 
        ${CMAKE_CURRENT_SOURCE_DIR}/libcan/libcan.a 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/App/libkey.so
        pthread
    )
    
endif()

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "ARX_CPP")
set(CPACK_PACKAGE_VERSION "0.0.1")
set(CPACK_PACKAGE_CONTACT "js011107@163.com")
set(CMAKE_BUILD_TYPE Debug)
include(CPack)
