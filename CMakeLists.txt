cmake_minimum_required( VERSION 3.0.2 )
project( Robdyn_tf )

set( SRC_DIR src )

add_compile_options( -std=c++0x )

#find_package( Eigen3 REQUIRED )
find_package( PkgConfig REQUIRED )
pkg_check_modules( ODE REQUIRED ode )
pkg_check_modules( OSGV REQUIRED openscenegraph-osgViewer )
pkg_check_modules( OSGS REQUIRED openscenegraph-osgShadow )
find_package( TensorflowCC REQUIRED ) # From https://github.com/FloopCZ/tensorflow_cc

include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${PROJECT_SOURCE_DIR}/${SRC_DIR} )


file( GLOB ROB_SOURCES ode/*.cc renderer/*.cc utils/*.cc )
#add_library( robdyn STATIC ${ROB_SOURCES} )
add_library( robdyn SHARED ${ROB_SOURCES} )
#target_include_directories( robdyn PUBLIC ${EIGEN3_INCLUDE_DIR} )
target_link_libraries( robdyn TensorflowCC::Shared ) # In order to use only TensorFlow's version of Eigen


add_executable( scene_1 ${SRC_DIR}/scene_1.cc
						${SRC_DIR}/rover_1.cc )
target_link_libraries( scene_1 robdyn
							   ${ODE_LIBRARIES}
							   ${OSGV_LIBRARIES}
							   ${OSGS_LIBRARIES} )

add_executable( ramp ${SRC_DIR}/ramp.cc
					 ${SRC_DIR}/rover_1.cc )
target_link_libraries( ramp robdyn
							${ODE_LIBRARIES}
							${OSGV_LIBRARIES}
							${OSGS_LIBRARIES} )



find_package( PythonLibs 3 REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )
find_package( Boost REQUIRED COMPONENTS python3 )
#find_package( Boost REQUIRED COMPONENTS python3 numpy3 )
include_directories( ${Boost_INCLUDE_DIR} )


set( ROVER_TRAINING_1_SOURCES ${SRC_DIR}/rover_training_1.cc
							  ${SRC_DIR}/rover_1_tf.cc
							  ${SRC_DIR}/rover_1.cc )

set( ROVER_TRAINING_1_LIBRARIES robdyn
								${ODE_LIBRARIES}
								${OSGV_LIBRARIES}
								${OSGS_LIBRARIES}
								TensorflowCC::Shared
								${Boost_LIBRARIES}
								${PYTHON_LIBRARIES} )

add_executable( rover_training_1_exe ${ROVER_TRAINING_1_SOURCES} )
target_link_libraries( rover_training_1_exe ${ROVER_TRAINING_1_LIBRARIES} )
target_compile_definitions( rover_training_1_exe PRIVATE EXE )
#target_compile_definitions( rover_training_1_exe PRIVATE PRINT )
#set_target_properties( rover_training_1_exe PROPERTIES COMPILE_FLAGS "-g" )

add_library( rover_training_1_module SHARED ${ROVER_TRAINING_1_SOURCES} )
target_link_libraries( rover_training_1_module ${ROVER_TRAINING_1_LIBRARIES} )
set_target_properties( rover_training_1_module PROPERTIES PREFIX "" )
set_target_properties( rover_training_1_module PROPERTIES LINK_FLAGS "-Wl,--no-undefined" )



#set_target_properties( robdyn scene_1 PROPERTIES COMPILE_FLAGS "-std=c++0x" )
#set_target_properties( robdyn scene_1 PROPERTIES LINK_FLAGS "-lX11" )
