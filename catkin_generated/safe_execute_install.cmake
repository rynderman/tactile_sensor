execute_process(COMMAND "/home/rynderman/ros_ws/src/tactile_sensor/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/rynderman/ros_ws/src/tactile_sensor/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
