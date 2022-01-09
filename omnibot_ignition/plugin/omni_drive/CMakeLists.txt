find_package(ignition-cmake2 REQUIRED)

set(IGN_COMMON_VER 4)
find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-plugin${IGN_PLUGIN_VER} REQUIRED COMPONENTS register)

set(IGN_GAZEBO_VER 6)
ign_find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

add_library(OmniDrive SHARED OmniDrive.cc)

set_property(TARGET OmniDrive PROPERTY CXX_STANDARD 17)

install(TARGETS OmniDrive DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

set(IGN_GAZEBO_SYSTEM_PLUGIN_PATH )

target_link_libraries(OmniDrive
  PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
