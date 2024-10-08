cmake_minimum_required(VERSION 3.10)
project(legged_unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
)
set(CycloneDDS_DIR "/home/rstaion/projects/nimesh/cyclonedds/install/lib/cmake/CycloneDDS")
set(CycloneDDS-CXX_DIR "/home/rstaion/projects/nimesh/cyclonedds-cxx/install/lib/cmake/CycloneDDS-CXX")

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

set(SOURCE src/legged_unitree_hw.cpp src/UnitreeHW.cpp)

# add_executable(${PROJECT_NAME}_3_3_1
#         ${SOURCE}
#         )

# target_compile_definitions(${PROJECT_NAME}_3_3_1 PRIVATE UNITREE_SDK_3_3_1)

# target_link_libraries(${PROJECT_NAME}_3_3_1
#         ${catkin_LIBRARIES}
#         libunitree_legged_sdk_amd64_3_3_1.so lcm
#         )

add_executable(${PROJECT_NAME}_3_8_0
        ${SOURCE}
)

target_compile_definitions(${PROJECT_NAME}_3_8_0 PRIVATE UNITREE_SDK_3_8_0)

target_link_libraries(${PROJECT_NAME}_3_8_0
        ${catkin_LIBRARIES}
        libunitree_legged_sdk_amd64_3_8_0.a
)

idlcxx_generate(TARGET unitree_go2_msgs 
        FILES
        include/dds/msgs/LowCmd.idl
        include/dds/msgs/LowState.idl
        WARNINGS no-implicit-extensibility
)

add_executable(${PROJECT_NAME}_go2
        src/legged_unitree_dds.cpp
        src/UnitreeDDS.cpp
)

target_compile_definitions(${PROJECT_NAME}_go2 PRIVATE UNITREE_SDK_3_8_0)

target_link_libraries(${PROJECT_NAME}_go2
        ${catkin_LIBRARIES}
        unitree_go2_msgs
        CycloneDDS-CXX::ddscxx
        libunitree_legged_sdk_amd64_3_8_0.a
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
        #     TARGETS ${PROJECT_NAME}_3_3_1 
            TARGETS ${PROJECT_NAME}_3_8_0 TARGETS ${PROJECT_NAME}_go2
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_unitree_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS 
        # ${PROJECT_NAME}_3_3_1 
        ${PROJECT_NAME}_3_8_0 ${PROJECT_NAME}_go2
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY 
        # include/${PROJECT_NAME}_3_3_1/ 
        include/${PROJECT_NAME}_3_8_0/ inlcude/dds/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
