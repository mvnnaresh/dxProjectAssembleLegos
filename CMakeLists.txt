cmake_minimum_required(VERSION 3.15)
project(dxProjectAssembleLegos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Tell CMake where to find our FindMuJoCo.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#--------------------------
#		 	QT
#--------------------------
if(NOT DEFINED ENV{QTDIR})
	MESSAGE(STATUS "QTDIR environment variable not set. Be sure to set the QTDIR cmake cache variable to the Qt root install location")
else()
	message(STATUS "Qt Dir Found: $ENV{QTDIR}")
endif()

find_package(Qt5 COMPONENTS Core Gui Network OpenGL Widgets QUIET)

if(Qt5_FOUND)
	set(QT_LIBRARIES ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5Widgets_LIBRARIES})
	set(QT_FOUND Qt5_FOUND)
else()
	message(STATUS "QT5 Not Found")
endif()

# Test for supported Qt version
find_program(QMAKE_EXECUTABLE NAMES qmake HINTS ${QTDIR} ENV QTDIR PATH_SUFFIXES bin)
execute_process(COMMAND ${QMAKE_EXECUTABLE} -query QT_VERSION OUTPUT_VARIABLE QT_VERSION)
if(QT_VERSION VERSION_LESS QT_MINIMUM_VERSION)
	MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}. Installed version: ${QT_VERSION}")
endif()
#--------------------------
#		 	EIGEN
#--------------------------
find_package (Eigen3 REQUIRED)
link_directories(${Eigen_INCLUDE_DIRS})

#--------------------------
#		 BOOST 
#--------------------------
find_package(Boost REQUIRED COMPONENTS thread filesystem date_time system serialization)
if(Boost_FOUND)
	message("[Message] >> Boost support found!!")
	set(Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIRS} CACHE INTERNAL "Boost include directories" FORCE)
	set(Boost_LIBRARIES ${Boost_LIBRARIES} CACHE INTERNAL "Boost libraries" FORCE)
else()
	message(FATAL_ERROR "Boost (essential) library not found!!") 
endif()

#--------------------------
#		 MUJOCO 
#--------------------------
find_package(MuJoCo REQUIRED)
if(MuJoCo_FOUND)
	message("[Message] >> MuJoCo support found!!")
else()
	message(FATAL_ERROR "[DexMan ERROR] >> MuJoCo support NOT found!!") 
endif()

#--------------------------
#		 OPENGL 
#--------------------------
find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)


#--------------------------
#		 ViSP 
#--------------------------
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
		find_package(VISP REQUIRED)
		if(VISP_FOUND)
			set(VISP_INCLUDE_DIRS ${VISP_INCLUDE_DIRS} CACHE INTERNAL "Path to VISP include directories" FORCE)
			set(VISP_LIBRARIES ${VISP_LIBRARIES} CACHE INTERNAL "VISP libraries" FORCE)
			message("[Message] >> ViSP support found!!")
		endif()
	else()
		message(WARNING "[DexMan WARNING] >> OpenCV Not found - so VISP support wont be added to DexMan!!")
endif()



add_executable(dxProjectAssembleLegos main.cpp
    demo.cpp
    demo.h
    dxMuJoCoRobotSimulator.cpp
    dxMuJoCoRobotSimulator.h
    dxMuJoCoRobotState.h
    dxKinMuJoCo.cpp
    dxKinMuJoCo.h
    dxPlannerSimple.cpp
    dxPlannerSimple.h
	mainwindow.h
	mainwindow.cpp
	mainwindow.ui
    dxMuJoCoRobotViewer.h
    dxMuJoCoRobotViewer.cpp)

target_include_directories(dxProjectAssembleLegos PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(dxProjectAssembleLegos PRIVATE
  MuJoCo::mujoco
  glfw
  OpenGL::GL
  ${Boost_LIBRARIES}
  ${VISP_LIBRARIES}
  ${QT_LIBRARIES}
  ${OpenCV_LIBS}
)


