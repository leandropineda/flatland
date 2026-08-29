# ament_target_dependencies() was removed from ament_cmake in ROS 2 Lyrical.
# Provide a minimal drop-in on distros where the command no longer exists, so
# the same CMakeLists builds from Humble through Rolling. Resolution order per
# dependency: modern namespaced target, rosidl ${pkg}_TARGETS, then the
# classic exported variables.
if(NOT COMMAND ament_target_dependencies)
  macro(ament_target_dependencies target)
    foreach(_dep ${ARGN})
      if(TARGET ${_dep}::${_dep})
        target_link_libraries(${target} ${_dep}::${_dep})
      elseif(DEFINED ${_dep}_TARGETS)
        target_link_libraries(${target} ${${_dep}_TARGETS})
      else()
        if(DEFINED ${_dep}_INCLUDE_DIRS)
          target_include_directories(${target} PUBLIC ${${_dep}_INCLUDE_DIRS})
        endif()
        if(DEFINED ${_dep}_LIBRARIES)
          target_link_libraries(${target} ${${_dep}_LIBRARIES})
        endif()
      endif()
    endforeach()
  endmacro()
endif()
