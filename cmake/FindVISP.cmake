# FindVISP.cmake
# Author: Naresh Marturi
# Date: 22/10/2023

if(NOT DEFINED ENV{VISP_DIR})
    message(FATAL_ERROR "Environment variable VISP_DIR is not set. Please set it to the root directory of ViSP installation.")
endif()

# Set VISP_DIR to the value from the environment variable
set(VISP_DIR $ENV{VISP_DIR})
message(STATUS "Using VISP_DIR from environment variable: ${VISP_DIR}")

# Set VISP_INCLUDE_DIR to the correct include directory
set(VISP_INCLUDE_DIR "${VISP_DIR}/include" )
if(EXISTS "${VISP_INCLUDE_DIR}")
    message(STATUS "Using VISP include directory: ${VISP_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "VISP include directory not found at: ${VISP_INCLUDE_DIR}")
endif()

# Cache VISP_INCLUDE_DIR to make it available in CMake GUI
set(VISP_INCLUDE_DIR "${VISP_INCLUDE_DIR}" CACHE PATH "Path to ViSP include directory" FORCE)

# Set the ViSP library directory based on the version structure
if(EXISTS "$ENV{VISP_DIR}/lib")
    # Newer versions with lib in standard location
    set(VISP_LIB_DIR "$ENV{VISP_DIR}/lib")
    message(STATUS "Using standard ViSP library directory: ${VISP_LIB_DIR}")
elseif(EXISTS "$ENV{VISP_DIR}/x64/vc15/lib")
    # Older versions with lib under x64/vc15/lib
    set(VISP_LIB_DIR "$ENV{VISP_DIR}/x64/vc15/lib")
    message(STATUS "Using older ViSP version library directory: ${VISP_LIB_DIR}")
else()
    # Fallback to dynamic search for .lib files
    file(GLOB_RECURSE VISP_LIBRARY_FILES "${VISP_DIR}/**/*.lib")
    if(VISP_LIBRARY_FILES)
        list(GET VISP_LIBRARY_FILES 0 FIRST_LIB_FILE)
        get_filename_component(VISP_LIB_DIR ${FIRST_LIB_FILE} DIRECTORY)
        message(STATUS "Dynamically found ViSP library directory: ${VISP_LIB_DIR}")
    else()
        message(FATAL_ERROR "No ViSP libraries found in ${VISP_DIR}")
    endif()
endif()

# Find all ViSP libraries by searching for .lib files in the lib directory
file(GLOB_RECURSE VISP_LIBRARY_FILES "${VISP_LIB_DIR}/*.lib")
set(VISP_LIBRARIES "")

foreach(LIBRARY_FILE ${VISP_LIBRARY_FILES})
    get_filename_component(LIB_NAME ${LIBRARY_FILE} NAME_WE)
    if(LIBRARY_FILE)
        list(APPEND VISP_LIBRARIES ${LIBRARY_FILE})
        message(STATUS "Looking for library: ${LIB_NAME} -- found")
    else()
        message(STATUS "Looking for library: ${LIB_NAME} -- not found")
    endif()
endforeach()

# Ensure that we found at least one library
if(VISP_INCLUDE_DIR AND NOT VISP_LIBRARIES STREQUAL "")
    # Define imported target VISP::VISP
    add_library(VISP::VISP INTERFACE IMPORTED)
    set_target_properties(VISP::VISP PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${VISP_INCLUDE_DIR}"
        #INTERFACE_INCLUDE_DIRECTORIES "${VISP_INCLUDE_DIR};${OpenCV_INCLUDE_DIRS}"
)
set_target_properties(VISP::VISP PROPERTIES
        INTERFACE_LINK_LIBRARIES "${VISP_LIBRARIES}"
		#INTERFACE_LINK_LIBRARIES "${VISP_LIBRARIES};${OpenCV_LIBRARIES};Eigen3::Eigen;${Boost_LIBRARIES};${PCL_LIBRARIES}"
)
    message(STATUS "VISP::VISP target created successfully in FindVISP.cmake.")
else()
    message(FATAL_ERROR "Failed to create VISP::VISP target due to missing include or library.")
endif()


# Cache VISP_LIBRARIES to make it available in CMake GUI with FORCE
if(NOT VISP_LIBRARIES STREQUAL "")
    set(VISP_LIBRARIES "${VISP_LIBRARIES}" CACHE STRING "List of ViSP libraries" FORCE)
else()
    message(FATAL_ERROR "No ViSP libraries found in ${VISP_DIR}/lib")
endif()

# Set VISP_FOUND, visp_FOUND, visp_INCLUDE_DIRS, visp_LIBRARIES, VISP_INCLUDE_DIRS, and VISP_LIBRARIES
set(VISP_FOUND TRUE)
set(visp_FOUND TRUE)
set(visp_INCLUDE_DIRS ${VISP_INCLUDE_DIR})
set(VISP_INCLUDE_DIRS ${VISP_INCLUDE_DIR})
set(VISP_LIBRARIES ${VISP_LIBRARIES})
set(visp_LIBRARIES ${VISP_LIBRARIES})

# Mark variables as advanced
mark_as_advanced(
    VISP_INCLUDE_DIR
    visp_INCLUDE_DIRS
    VISP_LIBRARIES
    visp_LIBRARIES
    VISP_INCLUDE_DIRS
)

# Print the used VISP include directory
message(STATUS "Used VISP_INCLUDE_DIR: ${VISP_INCLUDE_DIR}")
