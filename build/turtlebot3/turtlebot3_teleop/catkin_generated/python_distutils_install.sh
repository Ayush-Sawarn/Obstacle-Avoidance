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

echo_and_run cd "/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ayush/DL_Hackathon/Deimos_task1/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ayush/DL_Hackathon/Deimos_task1/install/lib/python3/dist-packages:/home/ayush/DL_Hackathon/Deimos_task1/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ayush/DL_Hackathon/Deimos_task1/build" \
    "/usr/bin/python3" \
    "/home/ayush/DL_Hackathon/Deimos_task1/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/ayush/DL_Hackathon/Deimos_task1/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ayush/DL_Hackathon/Deimos_task1/install" --install-scripts="/home/ayush/DL_Hackathon/Deimos_task1/install/bin"
