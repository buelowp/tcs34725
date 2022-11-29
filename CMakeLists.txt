cmake_minimum_required (VERSION 3.0.0)
project (tcs34725)

option (BUILD_TESTS "Build Google test capability (default OFF)" OFF)
option (TCS34725_BUILD_SHARED "Build as a shared object (default ON)" ON)

file (GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file (GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

set (CMAKE_CXX_STANDARD 14)

set(PROJ_VERSION_MAJOR 1)
set(PROJ_VERSION_MINOR 0)
set(PROJ_VERSION_PATCH 0)
set(PROJ_VERSION ${PROJ_VERSION_MAJOR}.${PROJ_VERSION_MINOR}.${PROJ_VERSION_PATCH} )

if(SHT31D_BUILD_SHARED)
    add_library (${PROJECT_NAME} SHARED ${SOURCES})
    set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJ_VERSION} SOVERSION ${PROJ_VERSION_MAJOR} )
else()
    add_library (${PROJECT_NAME} STATIC ${SOURCES})
endif()

install (TARGETS ${PROJECT_NAME} DESTINATION lib)
install (FILES ${HEADERS} DESTINATION include)

if (BUILD_TESTS)
    enable_testing()
    find_package (GTest REQUIRED)
    find_package(Threads REQUIRED)

    file (GLOB TESTS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")

    if(THREADS_HAVE_PTHREAD_ARG)
        target_compile_options(${PROJECT_NAME}-test PUBLIC "-pthread")
    endif()

    add_executable(${PROJECT_NAME}-test ${TESTS} ${HEADERS})
    target_link_libraries(${PROJECT_NAME}-test ${GTEST_LIBRARIES} ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
endif()
