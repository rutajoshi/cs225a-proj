# - SAI2-URDF
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# create an executable
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CS225A_BINARY_DIR}/final_project)
ADD_EXECUTABLE (final_proj final_proj.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (simviz_fp simviz_fp.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (leg_trajectory leg_trajectory.cpp ${CS225A_COMMON_SOURCE})
ADD_EXECUTABLE (control_robot control_robot.cpp ${CS225A_COMMON_SOURCE})

# and link the library against the executable
TARGET_LINK_LIBRARIES (final_proj ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (simviz_fp ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (leg_trajectory ${CS225A_COMMON_LIBRARIES})
TARGET_LINK_LIBRARIES (control_robot ${CS225A_COMMON_LIBRARIES} ${SAI2-PRIMITIVES_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(APP_RESOURCE_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/resources)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY world.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY panda_arm.urdf panda_arm_hand.urdf DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY human_leg.urdf DESTINATION ${APP_RESOURCE_DIR})

