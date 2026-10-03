# txt_to_ros2bag

Converts the SAD book txt sensor log (e.g. `data/ch3/10.txt`) into a ROS 2 bag (mcap).

| Line in txt | Topic | Type |
|---|---|---|
| `IMU t gx gy gz ax ay az` | `/imu` | `sensor_msgs/msg/Imu` |
| `GNSS t lat lon alt heading heading_valid` | `/gnss` | `sad_msgs/msg/Gnss` |
| `ODOM t left right` | `/odom` | `sad_msgs/msg/WheelPulse` |

`header.stamp` and the bag record time are both the timestamp from the txt file.

## Usage

1. Edit `param/txt_to_ros2bag.yaml` (`txt_path`, `output_bag`).
2. Build:

   ```
   colcon build --symlink-install --packages-select sad_msgs txt_to_ros2bag
   ```

3. Convert:

   ```
   source ./install/setup.bash
   ros2 launch txt_to_ros2bag txt_to_ros2bag_launch.py
   ```

4. Play (the workspace must be sourced so the `sad_msgs` types resolve):

   ```
   ros2 bag play sad_ch3_10_bag --clock --start-paused
   ```

   Start the consumer node with `use_sim_time:=true`, then press space to begin playback.
   Starting paused keeps the initial static segment intact for the static IMU initializer.
