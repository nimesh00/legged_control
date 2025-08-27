cmake_minimum_required(VERSION 3.10)
project(legged_xterra_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)
# set(CycloneDDS_DIR "/home/rstaion/projects/nimesh/cyclonedds/install/lib/cmake/CycloneDDS")
# set(CycloneDDS-CXX_DIR "/home/rstaion/projects/nimesh/cyclonedds-cxx/install/lib/cmake/CycloneDDS-CXX")

# list(APPEND /home/rstaion/projects/nimesh/cyclonedds-cxx/install/ CMAKE_PREFIX_PATH)

find_package(CycloneDDS-CXX REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

set(SOURCE src/legged_xterra_hw.cpp src/xTerraHW.cpp)

file(GLOB XTERRA_MSG_FILES msgs/idl/*.idl)
idlcxx_generate(TARGET xterra_dds_msgs
        FILES
        ${XTERRA_MSG_FILES}
        WARNINGS no-implicit-extensibility
)

add_executable(${PROJECT_NAME}
        ${SOURCE}
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        xterra_dds_msgs
        CycloneDDS-CXX::ddscxx
)

add_executable(keyboard_interface_bytearray src/keyboard_interface_bytearray.cpp)
target_link_libraries(keyboard_interface_bytearray
        xterra_dds_msgs
        CycloneDDS-CXX::ddscxx
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_xterra_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_xterra_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS 
        ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY 
        include/${PROJECT_NAME} inlcude/dds/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
