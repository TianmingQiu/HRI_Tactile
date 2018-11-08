cmake_minimum_required(VERSION 2.8.3)
project(skin_nao)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tum_ics_params
    tum_ics_tfs
    tum_ics_skin_msgs
    tum_ics_skin_bridge
    tum_ics_skin_descr
)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

catkin_package(
   CATKIN_DEPENDS roscpp tum_ics_params tum_ics_tfs tum_ics_skin_msgs tum_ics_skin_bridge tum_ics_skin_descr
   DEPENDS system_lib libqt4-dev
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})
message(STATUS "EIGEN3_INCLUDE_DIR " ${EIGEN3_INCLUDE_DIR})

find_package(TCommon REQUIRED)
add_definitions(${TCOMMON_DEFINITIONS})
include_directories(${TCOMMON_INCLUDE_DIRS})
message(STATUS "TCOMMON_INCLUDE_DIRS   " ${TCOMMON_INCLUDE_DIRS})
#message(STATUS "TCOMMON_LIBRARIES      " ${TCOMMON_LIBRARIES})

find_package(SkinCore REQUIRED)
include_directories(${SKINCORE_INCLUDE_DIRS})
add_definitions(${SKINCORE_DEFINITIONS})
message(STATUS "SKINCORE_INCLUDE_DIRS   " ${SKINCORE_INCLUDE_DIRS})
#message(STATUS "SKINCORE_LIBRARIES      " ${SKINCORE_LIBRARIES})


find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)

# add Qt Modules
set(QT_USE_QTNETWORK TRUE)

include(${QT_USE_FILE})

# DON'T USE (enforces Q_SIGNAL etc. macros)
# use here because of boost problems in tf etc.
# ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${roscpp_INCLUDE_DIRS}
)


# needed for Qt generated files
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

set(APP_TARGETS
    load_and_view_patch_tut_fiad
    load_and_view_patches_tut_fiad
)

foreach(target ${APP_TARGETS})
    add_executable(${target} src/Applications/main_${target}.cpp)
    add_dependencies(${target} ${tum_ics_skin_msgs_EXPORTED_TARGETS})

    target_link_libraries(${target}
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${SKINCORE_LIBRARIES}
        ${TCOMMON_LIBRARIES}
        ${QT_LIBRARIES}
    )
endforeach()


install(TARGETS ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)







