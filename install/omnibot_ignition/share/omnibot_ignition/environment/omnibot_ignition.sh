# generated from omnibot_ignition/env-hooks/omnibot_ignition.sh.in

ament_prepend_unique_value IGN_GAZEBO_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/models"
ament_prepend_unique_value IGN_GAZEBO_RESOURCE_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/worlds"

export MESA_GL_VERSION_OVERRIDE=3.3

# Deprecated

ament_prepend_unique_value SDF_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/models"
ament_prepend_unique_value SDF_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/worlds"

ament_prepend_unique_value IGN_FILE_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/models"
ament_prepend_unique_value IGN_FILE_PATH "$AMENT_CURRENT_PREFIX/share/omnibot_ignition/worlds"