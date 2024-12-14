# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multirobot_map_merge: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multirobot_map_merge_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_custom_target(_multirobot_map_merge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "multirobot_map_merge" "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" "nav_msgs/MapMetaData:geometry_msgs/Transform:geometry_msgs/Vector3:geometry_msgs/Pose:nav_msgs/OccupancyGrid:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(multirobot_map_merge
  "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_map_merge
)

### Generating Module File
_generate_module_cpp(multirobot_map_merge
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_map_merge
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multirobot_map_merge_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multirobot_map_merge_generate_messages multirobot_map_merge_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_dependencies(multirobot_map_merge_generate_messages_cpp _multirobot_map_merge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multirobot_map_merge_gencpp)
add_dependencies(multirobot_map_merge_gencpp multirobot_map_merge_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_map_merge_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(multirobot_map_merge
  "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multirobot_map_merge
)

### Generating Module File
_generate_module_eus(multirobot_map_merge
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multirobot_map_merge
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(multirobot_map_merge_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(multirobot_map_merge_generate_messages multirobot_map_merge_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_dependencies(multirobot_map_merge_generate_messages_eus _multirobot_map_merge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multirobot_map_merge_geneus)
add_dependencies(multirobot_map_merge_geneus multirobot_map_merge_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_map_merge_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(multirobot_map_merge
  "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_map_merge
)

### Generating Module File
_generate_module_lisp(multirobot_map_merge
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_map_merge
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multirobot_map_merge_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multirobot_map_merge_generate_messages multirobot_map_merge_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_dependencies(multirobot_map_merge_generate_messages_lisp _multirobot_map_merge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multirobot_map_merge_genlisp)
add_dependencies(multirobot_map_merge_genlisp multirobot_map_merge_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_map_merge_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(multirobot_map_merge
  "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multirobot_map_merge
)

### Generating Module File
_generate_module_nodejs(multirobot_map_merge
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multirobot_map_merge
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(multirobot_map_merge_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(multirobot_map_merge_generate_messages multirobot_map_merge_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_dependencies(multirobot_map_merge_generate_messages_nodejs _multirobot_map_merge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multirobot_map_merge_gennodejs)
add_dependencies(multirobot_map_merge_gennodejs multirobot_map_merge_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_map_merge_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(multirobot_map_merge
  "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_map_merge
)

### Generating Module File
_generate_module_py(multirobot_map_merge
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_map_merge
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multirobot_map_merge_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multirobot_map_merge_generate_messages multirobot_map_merge_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/chenyue001/Khattiya-Explore-Bench/src/map_merge/srv/mapPair2tf.srv" NAME_WE)
add_dependencies(multirobot_map_merge_generate_messages_py _multirobot_map_merge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(multirobot_map_merge_genpy)
add_dependencies(multirobot_map_merge_genpy multirobot_map_merge_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_map_merge_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_map_merge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_map_merge
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(multirobot_map_merge_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(multirobot_map_merge_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(multirobot_map_merge_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multirobot_map_merge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/multirobot_map_merge
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(multirobot_map_merge_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(multirobot_map_merge_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(multirobot_map_merge_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_map_merge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_map_merge
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(multirobot_map_merge_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(multirobot_map_merge_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(multirobot_map_merge_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multirobot_map_merge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/multirobot_map_merge
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(multirobot_map_merge_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(multirobot_map_merge_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(multirobot_map_merge_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_map_merge)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_map_merge\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_map_merge
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(multirobot_map_merge_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(multirobot_map_merge_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(multirobot_map_merge_generate_messages_py geometry_msgs_generate_messages_py)
endif()
