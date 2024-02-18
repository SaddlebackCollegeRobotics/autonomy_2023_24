while true; do
    current_date=$(date +%s)
    ros2 topic pub /base/fix sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: $current_date, nanosec: 0}, frame_id: 'odom'}, status: {status: 0, service: 0}, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
done
