# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wtr_serial: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iwtr_serial:/home/demphi/ros/quad_ws/src/wtr-serial/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wtr_serial_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_custom_target(_wtr_serial_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wtr_serial" "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wtr_serial
  "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wtr_serial
)

### Generating Services

### Generating Module File
_generate_module_cpp(wtr_serial
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wtr_serial
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wtr_serial_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wtr_serial_generate_messages wtr_serial_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_dependencies(wtr_serial_generate_messages_cpp _wtr_serial_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wtr_serial_gencpp)
add_dependencies(wtr_serial_gencpp wtr_serial_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wtr_serial_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(wtr_serial
  "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wtr_serial
)

### Generating Services

### Generating Module File
_generate_module_eus(wtr_serial
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wtr_serial
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(wtr_serial_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(wtr_serial_generate_messages wtr_serial_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_dependencies(wtr_serial_generate_messages_eus _wtr_serial_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wtr_serial_geneus)
add_dependencies(wtr_serial_geneus wtr_serial_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wtr_serial_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wtr_serial
  "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wtr_serial
)

### Generating Services

### Generating Module File
_generate_module_lisp(wtr_serial
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wtr_serial
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wtr_serial_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wtr_serial_generate_messages wtr_serial_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_dependencies(wtr_serial_generate_messages_lisp _wtr_serial_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wtr_serial_genlisp)
add_dependencies(wtr_serial_genlisp wtr_serial_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wtr_serial_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(wtr_serial
  "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wtr_serial
)

### Generating Services

### Generating Module File
_generate_module_nodejs(wtr_serial
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wtr_serial
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(wtr_serial_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(wtr_serial_generate_messages wtr_serial_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_dependencies(wtr_serial_generate_messages_nodejs _wtr_serial_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wtr_serial_gennodejs)
add_dependencies(wtr_serial_gennodejs wtr_serial_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wtr_serial_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wtr_serial
  "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wtr_serial
)

### Generating Services

### Generating Module File
_generate_module_py(wtr_serial
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wtr_serial
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wtr_serial_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wtr_serial_generate_messages wtr_serial_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/demphi/ros/quad_ws/src/wtr-serial/msg/actor_1.msg" NAME_WE)
add_dependencies(wtr_serial_generate_messages_py _wtr_serial_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wtr_serial_genpy)
add_dependencies(wtr_serial_genpy wtr_serial_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wtr_serial_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wtr_serial)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wtr_serial
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(wtr_serial_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wtr_serial)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/wtr_serial
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(wtr_serial_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wtr_serial)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wtr_serial
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(wtr_serial_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wtr_serial)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/wtr_serial
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(wtr_serial_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wtr_serial)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wtr_serial\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wtr_serial
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(wtr_serial_generate_messages_py std_msgs_generate_messages_py)
endif()
