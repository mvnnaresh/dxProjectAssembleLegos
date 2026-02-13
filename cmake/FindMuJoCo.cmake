# cmake/FindMuJoCo.cmake
# Usage:
#   find_package(MuJoCo REQUIRED)
# Provides:
#   MuJoCo_FOUND
#   MuJoCo_INCLUDE_DIRS
#   MuJoCo_LIBRARIES
#   MuJoCo_DLL_DIR
#   Target: MuJoCo::mujoco

include(FindPackageHandleStandardArgs)

# Hints from env or CMAKE_PREFIX_PATH
set(_MUJOCO_HINTS "")
foreach(var MUJOCO_ROOT MUJOCO_DIR MUJOCO_PATH)
  if(DEFINED ENV{${var}})
    list(APPEND _MUJOCO_HINTS "$ENV{${var}}")
  endif()
endforeach()

# Some common Windows locations (kept minimal)
list(APPEND _MUJOCO_HINTS
  "C:/mujoco"
  "C:/Program Files/mujoco"
  "$ENV{USERPROFILE}/.mujoco"
)

# Include dir
find_path(MuJoCo_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${_MUJOCO_HINTS}
  PATH_SUFFIXES include
)

# Import lib
find_library(MuJoCo_LIBRARY
  NAMES mujoco mujoco.lib
  HINTS ${_MUJOCO_HINTS}
  PATH_SUFFIXES lib
)

# DLL dir for runtime (Windows)
find_path(MuJoCo_DLL_DIR
  NAMES mujoco.dll
  HINTS ${_MUJOCO_HINTS}
  PATH_SUFFIXES bin
)

find_package_handle_standard_args(MuJoCo
  REQUIRED_VARS MuJoCo_INCLUDE_DIR MuJoCo_LIBRARY
)

if(MuJoCo_FOUND)
  set(MuJoCo_INCLUDE_DIRS "${MuJoCo_INCLUDE_DIR}")
  set(MuJoCo_LIBRARIES "${MuJoCo_LIBRARY}")

  if(NOT TARGET MuJoCo::mujoco)
    add_library(MuJoCo::mujoco UNKNOWN IMPORTED)
    set_target_properties(MuJoCo::mujoco PROPERTIES
      IMPORTED_LOCATION "${MuJoCo_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MuJoCo_INCLUDE_DIR}"
    )
  endif()
endif()
