set(CUSTOM_MODULES_DIR "${CMAKE_CURRENT_LIST_DIR}/../modules")

file(GLOB subdirs "${CUSTOM_MODULES_DIR}/*")

foreach(dir ${subdirs})
  if(IS_DIRECTORY ${dir})
    # Check if this directory contains a CMakeLists.txt (indicating a module)
    if(EXISTS ${dir}/CMakeLists.txt)
      list(APPEND ZEPHYR_EXTRA_MODULES ${dir})
    endif()
  endif()
endforeach()

message("Registered custom modules: ${ZEPHYR_EXTRA_MODULES}")
