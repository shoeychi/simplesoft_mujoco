
set(MUJOCO_simplysoft_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

set(MUJOCO_simplysoft_SRCS
    HumanRobotPhysicalContact.cc
    register.cc
    SoftHelper.cc
    SoftDef.cc
    PhyscisEngineData.cc
)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/biulab/cmakes")

find_package(cuPhy REQUIRED)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
find_package(nlohmann_json REQUIRED)

# add include
add_library(HumanRobotPhysicalContact SHARED)
target_sources(HumanRobotPhysicalContact PRIVATE ${MUJOCO_simplysoft_SRCS})
target_include_directories(HumanRobotPhysicalContact PRIVATE ${cuPhy_INCLUDE_DIRS} ${MUJOCO_simplysoft_INCLUDE})

target_link_libraries(HumanRobotPhysicalContact PRIVATE mujoco ${cuPhy_LIBRARIES}) # add library
target_link_libraries(HumanRobotPhysicalContact PRIVATE nlohmann_json)

target_compile_options(
  HumanRobotPhysicalContact
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  HumanRobotPhysicalContact
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
