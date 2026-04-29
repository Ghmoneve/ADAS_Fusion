# Array of radar targets published per detection frame
std_msgs/Header header   # Timestamp in header.stamp, frame_id = radar_link
RadarTarget[] targets    # Detected targets, empty if none detected
