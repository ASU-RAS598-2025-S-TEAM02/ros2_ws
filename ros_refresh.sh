ros_refresh() {
    echo "Stopping ROS 2 daemon..."
    ros2 daemon stop

    echo "Clearing shared memory..."
    sudo rm -rf /dev/shm/*

    if [ "$(whoami)" = "ubuntu" ] && [ "$(hostname)" = "ubuntu" ]; then
        echo "Restarting TurtleBot4 service for ubuntu@ubuntu..."
        sudo systemctl stop turtlebot4.service
        sudo systemctl start turtlebot4.service
    else
        echo "Skipping TurtleBot4 service restart (not ubuntu@ubuntu)."
    fi

    echo "Starting ROS 2 daemon..."
    ros2 daemon start

    echo "robot_refresh complete."
}