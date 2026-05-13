The `src` folder is a symlink pointing to the `Arduino_ROS/src` folder.

If you ever need to recreate the symlink:

1. Navigate to the `PySerial/DAQ/` folder.
2. Create the symlink: `ln -s ../../../Arduino_ROS/src src`