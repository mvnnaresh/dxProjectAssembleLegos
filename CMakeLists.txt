cmake_minimum_required(VERSION 3.15)
project(dxProjectAssembleLegos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tell CMake where to find our FindMuJoCo.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

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
    dxRobotViewerFactory.cpp
    dxRobotViewerFactory.h
    demo.cpp
    demo.h
    dxRobotSimulator.cpp
    dxRobotSimulator.h
    dxKinMujo.cpp
    dxKinMujo.h)

target_include_directories(dxProjectAssembleLegos PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(dxProjectAssembleLegos PRIVATE
  MuJoCo::mujoco
  glfw
  OpenGL::GL
  ${Boost_LIBRARIES}
  ${VISP_LIBRARIES}
)


