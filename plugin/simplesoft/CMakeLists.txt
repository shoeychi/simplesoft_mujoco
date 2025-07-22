
set(MUJOCO_simplysoft_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../..
                              ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

set(MUJOCO_simplysoft_SRCS
    Soft.cc
    register.cc
    SoftHelper.cc
)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/biulab/cmakes")

find_package(cuPhy REQUIRED)

# add include
add_library(simplesoft SHARED)
target_sources(simplesoft PRIVATE ${MUJOCO_simplysoft_SRCS})
target_include_directories(simplesoft PRIVATE ${cuPhy_INCLUDE_DIRS} ${MUJOCO_simplysoft_INCLUDE})

target_link_libraries(simplesoft PRIVATE mujoco ${cuPhy_LIBRARIES}) # add library

target_compile_options(
  simplesoft
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  simplesoft
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
