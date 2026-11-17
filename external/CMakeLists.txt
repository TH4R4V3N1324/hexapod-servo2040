# Pimoroni Pico libraries (submodule). Only the libraries this firmware uses are pulled in: the
# repository's own CMakeLists.txt also builds every example, which needs its nested submodules.
set(PIMORONI_PICO_PATH ${CMAKE_CURRENT_LIST_DIR}/pimoroni-pico)
include(${PIMORONI_PICO_PATH}/libraries/servo2040/servo2040.cmake)
include(${PIMORONI_PICO_PATH}/drivers/button/button.cmake)
include(${PIMORONI_PICO_PATH}/drivers/analogmux/analogmux.cmake)
include(${PIMORONI_PICO_PATH}/drivers/analog/analog.cmake)

# Gait, behaviours, robot definitions and packets shared with hexapod-ros2 (float Scalars, bundled
# Eigen since this is a cross build): gait_core::gait_core, gait_core::behaviour, gait_core::robots
# (explorer) and gait_core::protocol
add_subdirectory(hexapod-gait-core)
target_compile_definitions(gait_core PUBLIC EIGEN_NO_DEBUG)
