# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "f1tenth_control: 1 messages, 0 services")

set(MSG_I_FLAGS "-If1tenth_control:/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(f1tenth_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_custom_target(_f1tenth_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "f1tenth_control" "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(f1tenth_control
  "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f1tenth_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(f1tenth_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f1tenth_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(f1tenth_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(f1tenth_control_generate_messages f1tenth_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_dependencies(f1tenth_control_generate_messages_cpp _f1tenth_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f1tenth_control_gencpp)
add_dependencies(f1tenth_control_gencpp f1tenth_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f1tenth_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(f1tenth_control
  "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f1tenth_control
)

### Generating Services

### Generating Module File
_generate_module_eus(f1tenth_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f1tenth_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(f1tenth_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(f1tenth_control_generate_messages f1tenth_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_dependencies(f1tenth_control_generate_messages_eus _f1tenth_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f1tenth_control_geneus)
add_dependencies(f1tenth_control_geneus f1tenth_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f1tenth_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(f1tenth_control
  "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f1tenth_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(f1tenth_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f1tenth_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(f1tenth_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(f1tenth_control_generate_messages f1tenth_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_dependencies(f1tenth_control_generate_messages_lisp _f1tenth_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f1tenth_control_genlisp)
add_dependencies(f1tenth_control_genlisp f1tenth_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f1tenth_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(f1tenth_control
  "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f1tenth_control
)

### Generating Services

### Generating Module File
_generate_module_nodejs(f1tenth_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f1tenth_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(f1tenth_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(f1tenth_control_generate_messages f1tenth_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_dependencies(f1tenth_control_generate_messages_nodejs _f1tenth_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f1tenth_control_gennodejs)
add_dependencies(f1tenth_control_gennodejs f1tenth_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f1tenth_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(f1tenth_control
  "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f1tenth_control
)

### Generating Services

### Generating Module File
_generate_module_py(f1tenth_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f1tenth_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(f1tenth_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(f1tenth_control_generate_messages f1tenth_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/aimslab/Desktop/f1tenth/f1tenth_ws/src/f1tenth_lab/msg/RaceInfo.msg" NAME_WE)
add_dependencies(f1tenth_control_generate_messages_py _f1tenth_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(f1tenth_control_genpy)
add_dependencies(f1tenth_control_genpy f1tenth_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS f1tenth_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f1tenth_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/f1tenth_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(f1tenth_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f1tenth_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/f1tenth_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(f1tenth_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f1tenth_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/f1tenth_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(f1tenth_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f1tenth_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/f1tenth_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(f1tenth_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f1tenth_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f1tenth_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/f1tenth_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(f1tenth_control_generate_messages_py std_msgs_generate_messages_py)
endif()
