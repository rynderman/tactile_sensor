#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/rynderman/ros_ws/src/tactile_sensor"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/rynderman/ros_ws/install/lib/python2.7/dist-packages:/home/rynderman/ros_ws/src/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rynderman/ros_ws/src" \
    "/usr/bin/python" \
    "/home/rynderman/ros_ws/src/tactile_sensor/setup.py" \
    build --build-base "/home/rynderman/ros_ws/src/tactile_sensor" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/rynderman/ros_ws/install" --install-scripts="/home/rynderman/ros_ws/install/bin"
