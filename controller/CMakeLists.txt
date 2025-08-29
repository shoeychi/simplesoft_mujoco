set(MUJOCO_hrpc_controller_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

set(MUJOCO_hrpc_controller_SRCS
    SittingPose.cc
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../lib)
find_package(nlohmann_json REQUIRED)
find_package(glfw3 REQUIRED)


add_executable(humanSittingPose)
target_sources(humanSittingPose PRIVATE ${MUJOCO_hrpc_controller_SRCS})
target_include_directories(humanSittingPose PRIVATE ${MUJOCO_hrpc_controller_INCLUDE})
target_link_libraries(humanSittingPose PRIVATE mujoco nlohmann_json glfw)

# add cuPhy library
#target_include_directories(humanSittingPose PRIVATE ${cuPhy_INCLUDE_DIRS} )
#target_link_libraries(humanSittingPose PRIVATE  ${cuPhy_LIBRARIES}) 
