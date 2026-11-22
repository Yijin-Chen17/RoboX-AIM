cmake_minimum_required(VERSION 3.8)
project(hub_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(chassis_controllers REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(robox_hub_controller
        src/robox_hub_controller.cpp
)
ament_target_dependencies(robox_hub_controller
        rclcpp
        std_msgs
        custom_msgs
        chassis_controllers
)
target_include_directories(robox_hub_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(robox_hub_controller
        Eigen3::Eigen
)
target_compile_features(robox_hub_controller PUBLIC c_std_99 cxx_std_17)

install(TARGETS robox_hub_controller
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

ament_package()
