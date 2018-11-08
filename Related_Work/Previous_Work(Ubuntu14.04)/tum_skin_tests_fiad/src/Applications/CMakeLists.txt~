cmake_minimum_required(VERSION 2.6)

project(TestApps CXX C)

# viz_test

set(TARGET_NAME right_arm)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)


set(TARGET_NAME front)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME front_right_leg)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME left_arm)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME back_left_leg)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME right_arm_upper)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME left_arm_upper)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME all_cells)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME back)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(TARGET_NAME hand)
add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
add_dependencies(${TARGET_NAME} ${tum_skin_msgs_fiad_EXPORTED_TARGETS})
target_link_libraries(${TARGET_NAME}
    ${tum_skin_bridge_fiad_LIBRARIES}
    ${QT_LIBRARIES}
)

set(APP_TARGETS
    right_arm
    left_arm
    front_right_leg
    back_left_leg
    front
    left_arm_upper
    right_arm_upper
    all_cells
    back
    hand
)

install(TARGETS ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
