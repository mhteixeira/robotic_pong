#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/pedro/catkin_ws/src/niryo_one_ros/niryo_one_user_interface"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pedro/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pedro/catkin_ws/install/lib/python2.7/dist-packages:/home/pedro/catkin_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pedro/catkin_ws/build" \
    "/usr/bin/python2" \
    "/home/pedro/catkin_ws/src/niryo_one_ros/niryo_one_user_interface/setup.py" \
     \
    build --build-base "/home/pedro/catkin_ws/build/niryo_one_ros/niryo_one_user_interface" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pedro/catkin_ws/install" --install-scripts="/home/pedro/catkin_ws/install/bin"
