# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/ubuntu/thesis_mqtt/src/serial_stm32/msg/uartUSB.msg' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [uint8_t] in package [serial_stm32] with paths [['/home/ubuntu/thesis_mqtt/src/serial_stm32/msg']]")
message(STATUS "serial_stm32: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iserial_stm32:/home/ubuntu/thesis_mqtt/src/serial_stm32/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(serial_stm32_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(serial_stm32
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_stm32
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(serial_stm32_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(serial_stm32_generate_messages serial_stm32_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(serial_stm32_gencpp)
add_dependencies(serial_stm32_gencpp serial_stm32_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_stm32_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(serial_stm32
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_stm32
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(serial_stm32_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(serial_stm32_generate_messages serial_stm32_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(serial_stm32_geneus)
add_dependencies(serial_stm32_geneus serial_stm32_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_stm32_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(serial_stm32
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_stm32
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(serial_stm32_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(serial_stm32_generate_messages serial_stm32_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(serial_stm32_genlisp)
add_dependencies(serial_stm32_genlisp serial_stm32_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_stm32_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(serial_stm32
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_stm32
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(serial_stm32_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(serial_stm32_generate_messages serial_stm32_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(serial_stm32_gennodejs)
add_dependencies(serial_stm32_gennodejs serial_stm32_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_stm32_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(serial_stm32
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_stm32
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(serial_stm32_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(serial_stm32_generate_messages serial_stm32_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(serial_stm32_genpy)
add_dependencies(serial_stm32_genpy serial_stm32_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS serial_stm32_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_stm32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/serial_stm32
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(serial_stm32_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_stm32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/serial_stm32
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(serial_stm32_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_stm32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/serial_stm32
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(serial_stm32_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_stm32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/serial_stm32
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(serial_stm32_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_stm32)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_stm32\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/serial_stm32
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(serial_stm32_generate_messages_py std_msgs_generate_messages_py)
endif()
