zephyr_include_directories(include)
# TODO: figure out what this does, it looks shady. Found it here: https://jdelaney.me/posts/zephyr-oot-modules/
list(APPEND SYSCALL_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SYSCALL_INCLUDE_DIRS ${SYSCALL_INCLUDE_DIRS} PARENT_SCOPE)
add_subdirectory_ifdef(CONFIG_PMW3389 drivers/sensor/pmw3389)
