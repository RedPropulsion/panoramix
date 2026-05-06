# Auto-discover and register custom modules with Zephyr build system
# This file is sourced by Zephyr when MODULE_EXT_ROOT points to this directory
# To add a new module: create a subdirectory in panoramix/modules/ with CMakeLists.txt and Kconfig

file(GLOB subdirs "${CMAKE_CURRENT_LIST_DIR}/*")

foreach(dir ${subdirs})
  if(IS_DIRECTORY ${dir})
    # Check if this directory contains a CMakeLists.txt (indicating a module)
    if(EXISTS ${dir}/CMakeLists.txt)
      get_filename_component(module_name ${dir} NAME)
      
      # Set CMake and Kconfig paths
      set(ZEPHYR_${module_name}_CMAKE_DIR "${dir}")
      set(ZEPHYR_${module_name}_KCONFIG "${dir}/Kconfig")
    endif()
  endif()
endforeach()