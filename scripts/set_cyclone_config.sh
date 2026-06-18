export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -z "$ROS_NETWORK_INTERFACE" ]; then
    export ROS_NETWORK_INTERFACE=enp0s31f6
fi
# Use /sys/class/net rather than `ip` so the check works without iproute2.
if [ ! -e "/sys/class/net/$ROS_NETWORK_INTERFACE" ]; then
    echo "ROS_NETWORK_INTERFACE '$ROS_NETWORK_INTERFACE' not found, falling back to 'lo'."
    export ROS_NETWORK_INTERFACE=lo
fi

# CycloneDDS (RoboStack/apt builds) does not expand ${ROS_NETWORK_INTERFACE}
# placeholders inside the XML, so resolve it here with sed and point
# CYCLONEDDS_URI at the generated file. Otherwise the interface name is empty
# and the ros2 daemon dies with "Nameless and address-less interface".
# Substitute both placeholder spellings for robustness against template drift.
sed -e "s|\${ROS_NETWORK_INTERFACE}|${ROS_NETWORK_INTERFACE}|g" \
    -e "s|\${NETWORK_INTERFACE}|${ROS_NETWORK_INTERFACE}|g" \
    /home/ros/ros2_ws/src/crisp_controllers_demos/config/cyclone_config.xml \
    > /tmp/cyclone_config_resolved.xml
export CYCLONEDDS_URI=file:///tmp/cyclone_config_resolved.xml

ros2 daemon stop && ros2 daemon start
