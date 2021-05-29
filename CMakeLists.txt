cmake_minimum_required( VERSION 3.0.2 )
project( Robdyn_tf )

set( SRC_DIR src )

add_compile_options( -std=c++0x )

find_package( Eigen3 REQUIRED )
find_package( PkgConfig REQUIRED )
pkg_check_modules( ODE REQUIRED ode )
pkg_check_modules( OSGV REQUIRED openscenegraph-osgViewer )
pkg_check_modules( OSGS REQUIRED openscenegraph-osgShadow )
find_package( yaml-cpp REQUIRED )

include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${PROJECT_SOURCE_DIR}/${SRC_DIR} )



##########
# ROBDYN #
##########

file( GLOB ROB_SOURCES ode/*.cc renderer/*.cc Filters/cpp/*.cc )
add_library( robdyn SHARED ${ROB_SOURCES} )
target_include_directories( robdyn PUBLIC ${EIGEN3_INCLUDE_DIR} )

##############
# TENSORFLOW #
##############

set( TF_BINDING_DIR scripts/MachineLearning/tf_cpp_binding )
add_library( tensorflow_binding SHARED ${TF_BINDING_DIR}/tf_cpp_binding.cc )
target_include_directories( tensorflow_binding PUBLIC ${TF_BINDING_DIR} )
target_link_libraries( tensorflow_binding tensorflow )

###########
# FILTERS #
###########

set( FILTERS_DIR scripts/ModelTree/cpp )
add_library( filters STATIC ${FILTERS_DIR}/model_tree.cc )
target_include_directories( filters PUBLIC ${FILTERS_DIR} )

################
# BOOST.PYTHON #
################

find_package( PythonLibs 3 REQUIRED )
include_directories( ${PYTHON_INCLUDE_DIRS} )
find_package( Boost REQUIRED COMPONENTS python3 )
include_directories( ${Boost_INCLUDE_DIR} )



###########
# scene_1 #
###########

add_executable( scene_1 ${SRC_DIR}/scene_1.cc
						${SRC_DIR}/rover_1.cc )
target_link_libraries( scene_1 robdyn
							   ${ODE_LIBRARIES}
							   ${OSGV_LIBRARIES}
							   ${OSGS_LIBRARIES} )



#######
# arm #
#######

add_executable( arm ${SRC_DIR}/arm.cc )
target_link_libraries( arm robdyn
						   ${ODE_LIBRARIES}
						   ${OSGV_LIBRARIES}
						   ${OSGS_LIBRARIES} )


########
# ramp #
########

add_executable( ramp ${SRC_DIR}/ramp.cc
					 ${SRC_DIR}/rover_1.cc )
target_link_libraries( ramp robdyn
							${ODE_LIBRARIES}
							${OSGV_LIBRARIES}
							${OSGS_LIBRARIES} )


##############
# scene_1_mt #
##############

add_executable( scene_1_mt ${SRC_DIR}/scene_1_mt.cc
						   ${SRC_DIR}/rover_1_mt.cc
						   ${SRC_DIR}/rover_1.cc )
target_link_libraries( scene_1_mt robdyn
								  ${ODE_LIBRARIES}
								  ${OSGV_LIBRARIES}
								  ${OSGS_LIBRARIES}
								  filters
								  yaml-cpp )
#target_compile_definitions( scene_1_mt PRIVATE PRINT_STATE_AND_ACTIONS )


############
# crawling #
############

add_executable( crawling ${SRC_DIR}/crawling.cc
						 ${SRC_DIR}/rover_1.cc
						 ${SRC_DIR}/rover_1_crawlers.cc )
target_link_libraries( crawling robdyn
							    ${ODE_LIBRARIES}
							    ${OSGV_LIBRARIES}
							    ${OSGS_LIBRARIES} )


####################
# rover_training_1 #
####################

set( ROVER_TRAINING_1_SOURCES ${SRC_DIR}/rover_training_1.cc
							  ${SRC_DIR}/rover_1_tf.cc
							  ${SRC_DIR}/rover_1.cc )

set( ROVER_TRAINING_1_LIBRARIES robdyn
								${ODE_LIBRARIES}
								${OSGV_LIBRARIES}
								${OSGS_LIBRARIES}
								tensorflow_binding
								${Boost_LIBRARIES}
								${PYTHON_LIBRARIES} )

add_executable( rover_training_1_exe ${ROVER_TRAINING_1_SOURCES} )
target_link_libraries( rover_training_1_exe ${ROVER_TRAINING_1_LIBRARIES} )
target_compile_definitions( rover_training_1_exe PRIVATE EXE )
#target_compile_definitions( rover_training_1_exe PRIVATE PRINT_STATE_AND_ACTIONS )
#set_target_properties( rover_training_1_exe PROPERTIES COMPILE_FLAGS "-g" )

add_library( rover_training_1_module SHARED ${ROVER_TRAINING_1_SOURCES} )
target_link_libraries( rover_training_1_module ${ROVER_TRAINING_1_LIBRARIES} )
set_target_properties( rover_training_1_module PROPERTIES PREFIX "" )
set_target_properties( rover_training_1_module PROPERTIES LINK_FLAGS "-Wl,--no-undefined" )


######################
# data_collection_tf #
######################

add_executable( data_collection_tf ${SRC_DIR}/data_collection_tf.cc
							       ${SRC_DIR}/rover_1_tf.cc
							       ${SRC_DIR}/rover_1.cc )
target_link_libraries( data_collection_tf ${ROVER_TRAINING_1_LIBRARIES} )
target_compile_definitions( data_collection_tf PRIVATE PRINT_TRANSITIONS )
