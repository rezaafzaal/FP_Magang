# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "FP_Magang: 3 messages, 0 services")

set(MSG_I_FLAGS "-IFP_Magang:/home/rezaafzaal/FP_Magang/src/FP_Magang/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(FP_Magang_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_custom_target(_FP_Magang_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "FP_Magang" "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" ""
)

get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_custom_target(_FP_Magang_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "FP_Magang" "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" ""
)

get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_custom_target(_FP_Magang_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "FP_Magang" "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang
)
_generate_msg_cpp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang
)
_generate_msg_cpp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang
)

### Generating Services

### Generating Module File
_generate_module_cpp(FP_Magang
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(FP_Magang_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(FP_Magang_generate_messages FP_Magang_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_cpp _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_cpp _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_cpp _FP_Magang_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(FP_Magang_gencpp)
add_dependencies(FP_Magang_gencpp FP_Magang_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS FP_Magang_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang
)
_generate_msg_eus(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang
)
_generate_msg_eus(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang
)

### Generating Services

### Generating Module File
_generate_module_eus(FP_Magang
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(FP_Magang_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(FP_Magang_generate_messages FP_Magang_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_eus _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_eus _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_eus _FP_Magang_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(FP_Magang_geneus)
add_dependencies(FP_Magang_geneus FP_Magang_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS FP_Magang_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang
)
_generate_msg_lisp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang
)
_generate_msg_lisp(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang
)

### Generating Services

### Generating Module File
_generate_module_lisp(FP_Magang
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(FP_Magang_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(FP_Magang_generate_messages FP_Magang_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_lisp _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_lisp _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_lisp _FP_Magang_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(FP_Magang_genlisp)
add_dependencies(FP_Magang_genlisp FP_Magang_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS FP_Magang_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang
)
_generate_msg_nodejs(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang
)
_generate_msg_nodejs(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang
)

### Generating Services

### Generating Module File
_generate_module_nodejs(FP_Magang
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(FP_Magang_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(FP_Magang_generate_messages FP_Magang_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_nodejs _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_nodejs _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_nodejs _FP_Magang_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(FP_Magang_gennodejs)
add_dependencies(FP_Magang_gennodejs FP_Magang_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS FP_Magang_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang
)
_generate_msg_py(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang
)
_generate_msg_py(FP_Magang
  "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang
)

### Generating Services

### Generating Module File
_generate_module_py(FP_Magang
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(FP_Magang_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(FP_Magang_generate_messages FP_Magang_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/PC2BS.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_py _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/BS2PC.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_py _FP_Magang_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rezaafzaal/FP_Magang/src/FP_Magang/msg/koordinat.msg" NAME_WE)
add_dependencies(FP_Magang_generate_messages_py _FP_Magang_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(FP_Magang_genpy)
add_dependencies(FP_Magang_genpy FP_Magang_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS FP_Magang_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/FP_Magang
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(FP_Magang_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(FP_Magang_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/FP_Magang
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(FP_Magang_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(FP_Magang_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/FP_Magang
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(FP_Magang_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(FP_Magang_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/FP_Magang
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(FP_Magang_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(FP_Magang_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/FP_Magang
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(FP_Magang_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(FP_Magang_generate_messages_py geometry_msgs_generate_messages_py)
endif()
