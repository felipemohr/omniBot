find_package(ignition-cmake2 REQUIRED)

set(IGN_COMMON_VER 3)
find_package(ignition-common${IGN_COMMON_VER} REQUIRED)

set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-plugin${IGN_PLUGIN_VER} REQUIRED COMPONENTS register)

set(IGN_GAZEBO_VER 3)
ign_find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

add_library(OmniDrive SHARED OmniDrive.cc include/OmniDriveOdometry.cc)

install(TARGETS OmniDrive DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})

include_directories(include)

target_link_libraries(OmniDrive
  PRIVATE ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)
