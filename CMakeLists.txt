cmake_minimum_required(VERSION 3.22)
project(groundgrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Non-ROS deps not picked up by ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

include_directories(include)

ament_auto_add_executable(${PROJECT_NAME}_node
    src/GroundGridNode.cpp
    src/GroundGrid.cpp
    src/GroundSegmentation.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${OpenCV_LIBS}
)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(PROGRAMS
    scripts/eval_groundpoint_classifier.py
    DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY param
    DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package()
