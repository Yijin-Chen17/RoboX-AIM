cmake_minimum_required(VERSION 3.8)
project(chassis_controllers)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RoboxChassisControl.msg"

        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

add_library(robox_control_core STATIC
        src/utils/robox/kinematics/mobed_kinematics.cpp
        src/utils/robox/estimation/state_estimator.cpp
        src/utils/robox/estimation/terrain_estimator.cpp
        src/utils/robox/estimation/contact_detector.cpp
        src/utils/robox/estimation/climb_geometry_estimator.cpp
        src/utils/robox/dynamics/srbd_model.cpp
        src/utils/robox/dynamics/grf_optimizer.cpp
        src/utils/robox/tasks/trajectory_generator.cpp
        src/utils/robox/tasks/normal_climb_controller.cpp
        src/utils/robox/tasks/task_controller.cpp
        src/utils/robox/controllers/driving_controller.cpp
        src/utils/robox/controllers/balance_controller.cpp
        src/utils/robox/robox_solver.cpp
)
target_include_directories(robox_control_core PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_options(robox_control_core PRIVATE -Wno-psabi)
target_link_libraries(robox_control_core PUBLIC Eigen3::Eigen)

rosidl_get_typesupport_target(
        cpp_typesupport_target
        ${PROJECT_NAME}
        "rosidl_typesupport_cpp"
)

add_executable(dm_lk_robox_chassis_controller
        src/dm_lk_robox_chassis_controller.cpp
)
ament_target_dependencies(dm_lk_robox_chassis_controller
        rclcpp
        std_msgs
        sensor_msgs
        custom_msgs
)
target_link_libraries(dm_lk_robox_chassis_controller
        robox_control_core
        ${cpp_typesupport_target}
        Eigen3::Eigen
)
target_include_directories(dm_lk_robox_chassis_controller PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
)
target_compile_features(dm_lk_robox_chassis_controller PUBLIC c_std_99 cxx_std_17)
target_compile_options(dm_lk_robox_chassis_controller PRIVATE -Wno-psabi)

install(TARGETS dm_lk_robox_chassis_controller
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
