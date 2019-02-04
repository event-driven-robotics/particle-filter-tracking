cmake_minimum_required(VERSION 3.5)

project(vParticleFilterTracker)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

option(USE_QTCREATOR "Add apps/drivers to QtCreator IDE" OFF)

find_package(YARP 3.0 REQUIRED COMPONENTS OS)
find_package(ICUBcontrib REQUIRED)
find_package(eventdriven REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

file(GLOB source src/vControlLoopDelay.cpp src/vParticle.cpp)
file(GLOB header include/vControlLoopDelay.h include/vParticle.h)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${eventdriven_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${source} ${header})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} eventdriven)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

yarp_install(FILES ${PROJECT_NAME}.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/eventdriven)

if(USE_QTCREATOR)
    add_custom_target(${PROJECT_NAME}_docs SOURCES ${PROJECT_NAME}.ini)
endif(USE_QTCREATOR)
